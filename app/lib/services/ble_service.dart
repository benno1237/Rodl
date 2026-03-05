import 'dart:async';
import 'dart:typed_data';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';
import '../models/settings.dart';
import '../models/gps_point.dart';

class BleService {
  static final String serviceUuid = '12345678-1234-1234-1234-123456789abc';
  static final String settingsUuid = '12345678-1234-1234-1234-123456789abd';
  static final String gpsDataUuid = '12345678-1234-1234-1234-123456789abe';

  BluetoothDevice? _device;
  BluetoothCharacteristic? _settingsCharacteristic;
  BluetoothCharacteristic? _gpsDataCharacteristic;
  StreamSubscription? _settingsSubscription;
  StreamSubscription? _gpsDataSubscription;
  final _settingsController = StreamController<Settings>.broadcast();
  final _gpsPointController = StreamController<GPSPoint>.broadcast();

  Stream<Settings> get settingsStream => _settingsController.stream;
  Stream<GPSPoint> get gpsPointStream => _gpsPointController.stream;

  Future<bool> connect() async {
    try {
      final devices = await FlutterBluePlus.scanResults.first;
      final scanResult = devices.firstWhere(
        (d) => d.device.platformName.toLowerCase().contains('rodl'),
        orElse: () => devices.first,
      );

      final device = scanResult.device;
      await device.connect(timeout: const Duration(seconds: 10));
      _device = device;

      final services = await device.discoverServices();
      for (var service in services) {
        if (service.uuid.str.toLowerCase().contains(
          serviceUuid.substring(0, 8).toLowerCase(),
        )) {
          for (var char in service.characteristics) {
            if (char.uuid.str.toLowerCase().contains(
              settingsUuid.substring(0, 8).toLowerCase(),
            )) {
              _settingsCharacteristic = char;
            } else if (char.uuid.str.toLowerCase().contains(
              gpsDataUuid.substring(0, 8).toLowerCase(),
            )) {
              _gpsDataCharacteristic = char;
            }
          }
        }
      }
      await _setupNotifications();
      return _settingsCharacteristic != null;
    } catch (e) {
      return false;
    }
  }

  Future<void> _setupNotifications() async {
    if (_settingsCharacteristic != null) {
      await _settingsCharacteristic!.setNotifyValue(true);
      _settingsSubscription = _settingsCharacteristic!.onValueReceived.listen((value) {
        final settings = Settings.fromBytes(value.toList());
        _settingsController.add(settings);
      });
    }

    if (_gpsDataCharacteristic != null) {
      await _gpsDataCharacteristic!.setNotifyValue(true);
      _gpsDataSubscription = _gpsDataCharacteristic!.onValueReceived.listen((value) {
        if (value.length == 48) {
          final point = GPSPoint.fromBytes(Uint8List.fromList(value));
          _gpsPointController.add(point);
        }
      });
    }
  }

  Future<void> requestSettings() async {
    if (_settingsCharacteristic == null) return;
    await _settingsCharacteristic!.write(Uint8List.fromList([0x01]));
  }

  Future<void> sendSettings(Settings settings) async {
    if (_settingsCharacteristic == null) return;
    await _settingsCharacteristic!.write(
      Uint8List.fromList(settings.toBytes()),
    );
  }

  Future<void> disconnect() async {
    _settingsSubscription?.cancel();
    _gpsDataSubscription?.cancel();
    await _device?.disconnect();
    _device = null;
    _settingsCharacteristic = null;
    _gpsDataCharacteristic = null;
  }

  void dispose() {
    _settingsSubscription?.cancel();
    _gpsDataSubscription?.cancel();
    _settingsController.close();
    _gpsPointController.close();
  }
}
