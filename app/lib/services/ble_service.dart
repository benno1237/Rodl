import 'dart:async';
import 'dart:convert';

import 'package:flutter/foundation.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';
import 'package:permission_handler/permission_handler.dart';

import '../models/settings.dart';
import '../models/gps_point.dart';

/// Parsed contents of the device status notification (command characteristic).
class BleDeviceStatus {
  final bool powerOn;
  final bool isLogging;
  final int rideIndex;
  final int millisSinceBoot;

  const BleDeviceStatus({
    required this.powerOn,
    required this.isLogging,
    required this.rideIndex,
    required this.millisSinceBoot,
  });

  factory BleDeviceStatus.fromBytes(List<int> bytes) {
    if (bytes.length < 6) {
      return const BleDeviceStatus(
        powerOn: false,
        isLogging: false,
        rideIndex: 0,
        millisSinceBoot: 0,
      );
    }
    return BleDeviceStatus(
      powerOn: (bytes[0] & 0x01) != 0,
      isLogging: (bytes[0] & 0x02) != 0,
      rideIndex: bytes[1],
      millisSinceBoot: (bytes[2] << 24) |
          (bytes[3] << 16) |
          (bytes[4] << 8) |
          bytes[5],
    );
  }
}

/// A Rodl board seen in a BLE scan (passive - no connection needed).
class DiscoveredBoard {
  /// MAC (Android) / CoreBluetooth UUID (iOS) of the board.
  final String remoteId;

  /// Advertised display name (== the sled's name shown in the app).
  final String name;

  /// True when the board advertises the in-use flag (someone is connected).
  final bool inUse;

  /// True when the board advertises the 16-bit Rodl marker service uuid.
  final bool hasMarker;

  final int rssi;

  /// True when the advertisement looks like a Rodl board: it carries the
  /// marker service uuid (always present in our firmware) or a Rodl name.
  bool get isSled =>
      hasMarker || name.toLowerCase().contains('rodl');

  const DiscoveredBoard({
    required this.remoteId,
    required this.name,
    required this.inUse,
    required this.hasMarker,
    required this.rssi,
  });

  factory DiscoveredBoard.fromScanResult(ScanResult r) {
    final adv = r.advertisementData;
    final markerData = adv.serviceData[BleService.markerGuid];
    final hasMarker =
        markerData != null || adv.serviceUuids.contains(BleService.markerGuid);
    final inUse =
        markerData != null && markerData.isNotEmpty && markerData[0] != 0;
    final name = adv.advName.isNotEmpty ? adv.advName : r.device.platformName;
    return DiscoveredBoard(
      remoteId: r.device.remoteId.str,
      name: name,
      inUse: inUse,
      hasMarker: hasMarker,
      rssi: r.rssi,
    );
  }
}

class BleService {
  static final String serviceUuid = '12345678-1234-1234-1234-123456789abc';
  static final String settingsUuid = '12345678-1234-1234-1234-123456789abd';
  static final String gpsDataUuid = '12345678-1234-1234-1234-123456789abe';
  static final String commandUuid = '12345678-1234-1234-1234-123456789abf';
  static final String nameUuid = '12345678-1234-1234-1234-1234567890bf';

  /// 16-bit marker service uuid the firmware advertises (with a 1-byte
  /// in-use flag as service data). The full 128-bit uuid does not fit in
  /// the advertisement alongside the name.
  static final String markerUuid =
      '0000524c-0000-1000-8000-00805f9b34fb';
  static final Guid markerGuid = Guid(markerUuid);

  static const int maxNameLength = 20;

  // Command bytes (written to the command characteristic).
  static const int cmdStartLog = 0x01;
  static const int cmdStopLog = 0x02;
  static const int cmdPowerOn = 0x03;
  static const int cmdPowerOff = 0x04;
  static const int cmdTimeSync = 0x05;
  static const int cmdGetStatus = 0x06;

  BluetoothDevice? _device;
  BluetoothCharacteristic? _settingsCharacteristic;
  BluetoothCharacteristic? _gpsDataCharacteristic;
  BluetoothCharacteristic? _commandCharacteristic;
  BluetoothCharacteristic? _nameCharacteristic;
  StreamSubscription? _settingsSubscription;
  StreamSubscription? _gpsDataSubscription;
  StreamSubscription? _commandSubscription;
  StreamSubscription? _connectionSubscription;
  final _settingsController = StreamController<Settings>.broadcast();
  final _gpsPointController = StreamController<GPSPoint>.broadcast();
  final _statusController = StreamController<BleDeviceStatus>.broadcast();
  final _connectionController = StreamController<bool>.broadcast();

  // Passive discovery (home screen).
  final _discoveryController =
      StreamController<List<DiscoveredBoard>>.broadcast();
  final _discoveryStateController = StreamController<bool>.broadcast();
  StreamSubscription? _discoverySub;
  Timer? _discoveryTimer;
  bool _discovering = false;

  // Our own last disconnect, used to tolerate a stale in-use flag right
  // after releasing a sled ourselves.
  String? _lastDisconnectAddr;
  DateTime? _lastDisconnectAt;

  /// Human readable reason for the last failed [connect] attempt, if any.
  String? lastError;

  /// `deviceMillis + deviceEpochOffset == wall clock epoch ms`.
  int? deviceEpochOffset;

  Stream<Settings> get settingsStream => _settingsController.stream;
  Stream<GPSPoint> get gpsPointStream => _gpsPointController.stream;
  Stream<BleDeviceStatus> get statusStream => _statusController.stream;
  Stream<bool> get connectionStream => _connectionController.stream;

  /// Live list of boards seen in the current discovery scan (RSSI sorted).
  Stream<List<DiscoveredBoard>> get discoveryStream =>
      _discoveryController.stream;

  /// Emits true when a discovery scan starts and false when it stops.
  Stream<bool> get discoveryStateStream => _discoveryStateController.stream;
  bool get isDiscovering => _discovering;

  BluetoothDevice? get device => _device;
  bool get isConnected => _device?.isConnected ?? false;

  /// Platform identifier of the connected device (MAC on Android, CoreBluetooth
  /// UUID on iOS). Stable across reconnects, so it is what we persist to bind a
  /// physical board to a sled in the app.
  /// Set right after a successful rename so consumers see the new name
  /// before the platform name cache catches up (cleared on disconnect).
  String? _nameOverride;
  String? get remoteId => _device?.remoteId.str;
  String? get remoteName => _nameOverride ??
      ((_device?.platformName.isNotEmpty ?? false)
          ? _device!.platformName
          : null);

  Future<bool> _ensurePermissions() async {
    // Android 12+ needs runtime grants for scan/connect. iOS shows its own
    // CoreBluetooth prompt on first use.
    if (defaultTargetPlatform != TargetPlatform.android) return true;

    final statuses = await [
      Permission.bluetoothScan,
      Permission.bluetoothConnect,
    ].request();
    final scan = statuses[Permission.bluetoothScan] ?? PermissionStatus.denied;
    final connect =
        statuses[Permission.bluetoothConnect] ?? PermissionStatus.denied;

    if (kDebugMode) {
      print('[BLE] scan permission: $scan, connect permission: $connect');
    }
    if (scan.isPermanentlyDenied || connect.isPermanentlyDenied) {
      lastError = 'Bluetooth permission denied.\nEnable "Nearby devices" for '
          'Rodl in the system app settings.';
      return false;
    }
    if (!(scan.isGranted || scan.isLimited) ||
        !(connect.isGranted || connect.isLimited)) {
      lastError = 'Bluetooth permission not granted.';
      return false;
    }
    return true;
  }

  /// Whether the Bluetooth radio is currently on.
  Future<bool> get isAdapterOn async =>
      await FlutterBluePlus.adapterState.first == BluetoothAdapterState.on;

  /// Makes sure the radio is on; on Android asks the system to enable it.
  Future<bool> ensureAdapterOn() async {
    if (await FlutterBluePlus.adapterState.first == BluetoothAdapterState.on) {
      return true;
    }
    if (kDebugMode) print('[BLE] adapter off, requesting turn on');
    if (defaultTargetPlatform == TargetPlatform.android) {
      try {
        await FlutterBluePlus.turnOn();
      } catch (e) {
        if (kDebugMode) print('[BLE] turnOn failed: $e');
      }
    }
    if (await FlutterBluePlus.adapterState.first == BluetoothAdapterState.on) {
      return true;
    }
    lastError = 'Bluetooth is off. Turn it on and retry.';
    return false;
  }

  /// Starts a passive scan for nearby boards. Results are emitted on
  /// [discoveryStream] as they arrive; the scan stops itself after [window].
  Future<bool> startDiscovery({
    Duration window = const Duration(seconds: 8),
  }) async {
    if (_discovering) return true;
    lastError = null;
    try {
      if (!await _ensurePermissions()) return false;
      if (!await ensureAdapterOn()) return false;

      _discovering = true;
      if (!_discoveryStateController.isClosed) {
        _discoveryStateController.add(true);
      }
      if (kDebugMode) {
        print('[BLE] discovery started (window ${window.inSeconds}s)');
      }

      _discoverySub = FlutterBluePlus.scanResults.listen((results) {
        if (_discoveryController.isClosed) return;
        final boards = results
            .map(DiscoveredBoard.fromScanResult)
            .where((b) => b.isSled)
            .toList()
          ..sort((a, b) => b.rssi.compareTo(a.rssi));
        _discoveryController.add(boards);
      });

      await FlutterBluePlus.startScan();
      _discoveryTimer = Timer(window, stopDiscovery);
      return true;
    } on FlutterBluePlusException catch (e) {
      lastError = 'BLE error: ${e.description ?? e.code}';
      if (kDebugMode) print('[BLE] discovery failed: $e');
      await stopDiscovery();
      return false;
    } catch (e) {
      lastError = 'Scan failed: $e';
      if (kDebugMode) print('[BLE] discovery failed: $e');
      await stopDiscovery();
      return false;
    }
  }

  Future<void> stopDiscovery() async {
    _discoveryTimer?.cancel();
    _discoveryTimer = null;
    if (!_discovering) return;
    _discovering = false;
    await _discoverySub?.cancel();
    _discoverySub = null;
    try {
      await FlutterBluePlus.stopScan();
    } catch (_) {}
    if (kDebugMode) print('[BLE] discovery stopped');
    if (!_discoveryStateController.isClosed) {
      _discoveryStateController.add(false);
    }
  }

  /// Whether we saw this board flagged as in-use in the latest scan, and it
  /// was not our own link that just dropped.
  bool _isInUseByOther(String address) {
    bool? seen;
    for (final r in FlutterBluePlus.lastScanResults) {
      if (r.device.remoteId.str != address) continue;
      seen = DiscoveredBoard.fromScanResult(r).inUse;
      break;
    }
    if (seen != true) return false;
    final lastAt = _lastDisconnectAt;
    if (_lastDisconnectAddr == address &&
        lastAt != null &&
        DateTime.now().difference(lastAt) < const Duration(seconds: 5)) {
      return false; // stale flag from our own connection
    }
    return true;
  }

  /// Connect to a Rodl sled.
  ///
  /// When [targetAddress] is provided, try to reconnect to that specific board
  /// directly (no scan). If that fails — or no address is given — fall back to
  /// scanning for any board advertising the Rodl marker or name.
  Future<bool> connect({String? targetAddress}) async {
    if (isConnected) return true;
    lastError = null;
    // A connect scan would stop (and be stopped by) our own discovery scan.
    await stopDiscovery();
    try {
      if (!await _ensurePermissions()) return false;
      if (!await ensureAdapterOn()) return false;

      BluetoothDevice? device;

      // Direct reconnect to a previously paired board.
      if (targetAddress != null && targetAddress.isNotEmpty) {
        if (_isInUseByOther(targetAddress)) {
          lastError = 'In use by another user.';
          return false;
        }
        if (kDebugMode) print('[BLE] reconnecting to $targetAddress');
        try {
          device = BluetoothDevice.fromId(targetAddress);
          await device.connect(
            license: License.nonprofit,
            timeout: const Duration(seconds: 10),
            mtu: 512,
          );
        } catch (e) {
          if (kDebugMode) print('[BLE] direct connect failed: $e');
          device = null; // stale / out of range — fall through to a scan
        }
      }

      // Scan for the board.
      //
      // Note: in flutter_blue_plus 2.x `startScan`'s future returns as soon as
      // the scan has *started*, not when it has finished - the `timeout:`
      // argument only arms an internal timer. We therefore drive the scan
      // window ourselves and stop early once a confident match (Rodl marker,
      // not in use) shows up.
      if (device == null || !device.isConnected) {
        const window = Duration(seconds: 8);
        if (kDebugMode) {
          print('[BLE] scanning up to ${window.inSeconds}s for a Rodl board');
        }

        final found = Completer<ScanResult?>();
        final sub = FlutterBluePlus.scanResults.listen((results) {
          if (found.isCompleted) return;
          for (final r in results) {
            final board = DiscoveredBoard.fromScanResult(r);
            if (board.hasMarker && !board.inUse) {
              found.complete(r);
              return;
            }
          }
        });

        ScanResult? match;
        try {
          await FlutterBluePlus.startScan();
          match = await found.future.timeout(window, onTimeout: () => null);
        } finally {
          sub.cancel();
          await FlutterBluePlus.stopScan();
        }

        final results = FlutterBluePlus.lastScanResults;
        if (kDebugMode) {
          print('[BLE] scan finished, saw ${results.length} device(s):');
          for (final r in results) {
            final b = DiscoveredBoard.fromScanResult(r);
            print('[BLE]   ${b.remoteId} "${b.name}" rssi=${b.rssi} '
                'marker=${b.hasMarker} inUse=${b.inUse}');
          }
        }
        match ??= _pickScanMatch(results);
        if (match == null) {
          lastError = results.isEmpty
              ? 'No BLE devices found nearby. Is the sled powered on?'
              : 'Saw ${results.length} device(s) but no Rodl sled. '
                  'Is the sled powered on and running Rodl firmware?';
          return false;
        }
        if (_isInUseByOther(match.device.remoteId.str)) {
          lastError = 'In use by another user.';
          return false;
        }

        if (kDebugMode) {
          print('[BLE] connecting to ${match.device.remoteId.str} '
              '"${match.device.platformName}"');
        }
        device = match.device;
        await device.connect(
          license: License.nonprofit,
          timeout: const Duration(seconds: 10),
          mtu: 512,
        );
      }

      _device = device;
      deviceEpochOffset = null;

      _connectionSubscription?.cancel();
      _connectionSubscription = device.connectionState.listen((state) {
        final connected = state == BluetoothConnectionState.connected;
        if (kDebugMode) print('[BLE] connectionState: $state');
        if (!connected) {
          _lastDisconnectAddr = _device?.remoteId.str;
          _lastDisconnectAt = DateTime.now();
        }
        _connectionController.add(connected);
        if (!connected) {
          _teardownCharacteristics();
        }
      });

      final services = await device.discoverServices();
      for (var service in services) {
        if (service.uuid != Guid(serviceUuid)) continue;
        for (var char in service.characteristics) {
          if (char.uuid == Guid(settingsUuid)) {
            _settingsCharacteristic = char;
          } else if (char.uuid == Guid(gpsDataUuid)) {
            _gpsDataCharacteristic = char;
          } else if (char.uuid == Guid(commandUuid)) {
            _commandCharacteristic = char;
          } else if (char.uuid == Guid(nameUuid)) {
            _nameCharacteristic = char;
          }
        }
      }
      await _setupNotifications();

      final ok =
          _settingsCharacteristic != null && _commandCharacteristic != null;
      if (!ok) {
        // Device does not speak our protocol - drop the half connection so a
        // later attempt starts from scratch.
        lastError = 'Found a board, but it is not running Rodl firmware.';
        _teardownCharacteristics();
        await device.disconnect();
        _device = null;
      } else if (kDebugMode) {
        print('[BLE] connected to ${device.remoteId.str} '
            '"${device.platformName}"');
      }
      return ok;
    } on FlutterBluePlusException catch (e) {
      lastError = 'BLE error: ${e.description ?? e.code}';
      if (kDebugMode) print('[BLE] FlutterBluePlusException: $e');
      return false;
    } catch (e) {
      lastError = 'Connection failed: $e';
      if (kDebugMode) print('[BLE] connect failed: $e');
      return false;
    }
  }

  /// Best candidate from a finished scan: Rodl marker first (in-use boards
  /// last), then boards whose name contains "rodl".
  ScanResult? _pickScanMatch(List<ScanResult> results) {
    ScanResult? best;
    var bestScore = 0;
    for (final r in results) {
      final board = DiscoveredBoard.fromScanResult(r);
      final name = board.name.toLowerCase();
      final score = switch ((board.hasMarker, board.inUse, name.contains('rodl'))) {
        (true, false, _) => 4,
        (false, false, true) => 3,
        (true, true, _) => 2,
        (false, true, true) => 1,
        _ => 0,
      };
      if (score > bestScore) {
        bestScore = score;
        best = r;
      }
    }
    return bestScore > 0 ? best : null;
  }

  Future<void> _setupNotifications() async {
    if (_settingsCharacteristic != null) {
      await _settingsCharacteristic!.setNotifyValue(true);
      _settingsSubscription =
          _settingsCharacteristic!.onValueReceived.listen((value) {
        final settings = Settings.fromBytes(value.toList());
        _settingsController.add(settings);
      });
    }

    if (_gpsDataCharacteristic != null) {
      await _gpsDataCharacteristic!.setNotifyValue(true);
      _gpsDataSubscription =
          _gpsDataCharacteristic!.onValueReceived.listen((value) {
        if (value.length == 48) {
          final point = GPSPoint.fromBytes(Uint8List.fromList(value));
          _gpsPointController.add(point);
        }
      });
    }

    if (_commandCharacteristic != null) {
      await _commandCharacteristic!.setNotifyValue(true);
      _commandSubscription =
          _commandCharacteristic!.onValueReceived.listen((value) {
        final status = BleDeviceStatus.fromBytes(value.toList());
        _statusController.add(status);
      });
    }
  }

  void _teardownCharacteristics() {
    _settingsSubscription?.cancel();
    _gpsDataSubscription?.cancel();
    _commandSubscription?.cancel();
    _settingsSubscription = null;
    _gpsDataSubscription = null;
    _commandSubscription = null;
    _settingsCharacteristic = null;
    _gpsDataCharacteristic = null;
    _commandCharacteristic = null;
    _nameCharacteristic = null;
    deviceEpochOffset = null;
    _nameOverride = null;
  }

  /// Current display name of the connected board (reads the name
  /// characteristic), or null when unavailable.
  Future<String?> readDeviceName() async {
    if (_nameCharacteristic == null) return null;
    try {
      final bytes = await _nameCharacteristic!.read();
      final name = utf8.decode(bytes, allowMalformed: true).trim();
      return name.isEmpty ? null : name;
    } catch (_) {
      return null;
    }
  }

  /// Renames the connected board. The firmware persists the name to NVS and
  /// restarts advertising under it.
  Future<bool> writeDeviceName(String name) async {
    if (_nameCharacteristic == null) return false;
    var sanitized = name.trim();
    if (sanitized.length > maxNameLength) {
      sanitized = sanitized.substring(0, maxNameLength);
    }
    if (sanitized.isEmpty) return false;
    try {
      await _nameCharacteristic!.write(Uint8List.fromList(utf8.encode(sanitized)));
      if (kDebugMode) print('[BLE] wrote device name "$sanitized"');
      _nameOverride = sanitized;
      return true;
    } catch (e) {
      if (kDebugMode) print('[BLE] writeDeviceName failed: $e');
      return false;
    }
  }

  Future<void> requestSettings() async {
    if (_settingsCharacteristic == null) return;
    await _settingsCharacteristic!.write(Uint8List.fromList([0x01]));
  }

  /// Slider drags fire on every touch move; writing each one straight to the
  /// characteristic floods the link (the sled used to fall back to ~1 Hz
  /// because every write also committed flash). Coalesce instead: first
  /// change goes out immediately, subsequent ones collapse into at most one
  /// write per [_settingsMinInterval] carrying the newest payload - so the
  /// final value of a drag always lands shortly after the finger lifts.
  Timer? _settingsFlushTimer;
  DateTime? _lastSettingsSentAt;
  Settings? _pendingSettings;
  static const Duration _settingsMinInterval = Duration(milliseconds: 100);

  Future<void> sendSettings(Settings settings) async {
    if (_settingsCharacteristic == null) return;
    final now = DateTime.now();
    final last = _lastSettingsSentAt;
    if (last == null || now.difference(last) >= _settingsMinInterval) {
      _lastSettingsSentAt = now;
      await _writeSettings(settings);
      return;
    }
    _pendingSettings = settings; // newest wins
    _settingsFlushTimer ??=
        Timer(_settingsMinInterval - now.difference(last), () {
      _settingsFlushTimer = null;
      final s = _pendingSettings;
      _pendingSettings = null;
      if (s == null) return;
      _lastSettingsSentAt = DateTime.now();
      _writeSettings(s);
    });
  }

  Future<void> _writeSettings(Settings settings) async {
    if (_settingsCharacteristic == null) return;
    try {
      await _settingsCharacteristic!.write(Uint8List.fromList(settings.toBytes()));
    } catch (e) {
      if (kDebugMode) print('[BLE] sendSettings failed: $e');
    }
  }

  Future<bool> sendCommand(int command, [List<int>? payload]) async {
    if (_commandCharacteristic == null) return false;
    try {
      await _commandCharacteristic!.write(
        Uint8List.fromList([command, ...?payload]),
      );
      return true;
    } catch (_) {
      return false;
    }
  }

  Future<BleDeviceStatus?> getStatus({
    Duration timeout = const Duration(seconds: 2),
  }) async {
    if (_commandCharacteristic == null) return null;
    final future = _statusController.stream.first.timeout(
      timeout,
      onTimeout: () => throw TimeoutException('No status received'),
    );
    final ok = await sendCommand(cmdGetStatus);
    if (!ok) {
      future.ignore();
      return null;
    }
    try {
      return await future;
    } catch (_) {
      return null;
    }
  }

  /// Ask the device for its `millis()` and compute the offset that converts
  /// device timestamps into wall-clock epoch milliseconds.
  Future<int?> timeSync({
    Duration timeout = const Duration(seconds: 2),
  }) async {
    if (_commandCharacteristic == null) return null;
    final future = _statusController.stream.first.timeout(
      timeout,
      onTimeout: () => throw TimeoutException('No time sync response'),
    );
    final ok = await sendCommand(cmdTimeSync);
    if (!ok) {
      future.ignore();
      return deviceEpochOffset;
    }
    try {
      final status = await future;
      deviceEpochOffset =
          DateTime.now().millisecondsSinceEpoch - status.millisSinceBoot;
      return deviceEpochOffset;
    } catch (_) {
      return deviceEpochOffset;
    }
  }

  Future<void> disconnect() async {
    _teardownCharacteristics();
    _connectionSubscription?.cancel();
    _connectionSubscription = null;
    final addr = _device?.remoteId.str;
    await _device?.disconnect();
    if (addr != null) {
      _lastDisconnectAddr = addr;
      _lastDisconnectAt = DateTime.now();
    }
    _device = null;
    if (!_connectionController.isClosed) _connectionController.add(false);
  }

  void dispose() {
    _discoveryTimer?.cancel();
    _settingsFlushTimer?.cancel();
    _discoverySub?.cancel();
    _discovering = false;
    _teardownCharacteristics();
    _connectionSubscription?.cancel();
    _settingsController.close();
    _gpsPointController.close();
    _statusController.close();
    _connectionController.close();
    _discoveryController.close();
    _discoveryStateController.close();
  }
}
