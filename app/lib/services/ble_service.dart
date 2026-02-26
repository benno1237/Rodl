import 'dart:async';
import 'dart:typed_data';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';
import '../models/settings.dart';

class BleService {
  static final String serviceUuid = '12345678-1234-1234-1234-123456789abc';
  static final String settingsUuid = '12345678-1234-1234-1234-123456789abd';

  BluetoothDevice? _device;
  BluetoothCharacteristic? _settingsCharacteristic;
  StreamSubscription? _subscription;
  final _settingsController = StreamController<Settings>.broadcast();

  Stream<Settings> get settingsStream => _settingsController.stream;

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
              await _setupNotifications();
              return true;
            }
          }
        }
      }
      return false;
    } catch (e) {
      return false;
    }
  }

  Future<void> _setupNotifications() async {
    await _settingsCharacteristic?.setNotifyValue(true);
    _subscription = _settingsCharacteristic?.onValueReceived.listen((value) {
      final settings = Settings.fromBytes(value.toList());
      _settingsController.add(settings);
    });
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
    _subscription?.cancel();
    await _device?.disconnect();
    _device = null;
    _settingsCharacteristic = null;
  }

  void dispose() {
    _subscription?.cancel();
    _settingsController.close();
  }
}
