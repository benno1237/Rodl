import 'dart:async';
import 'package:flutter/foundation.dart';
import '../models/settings.dart';
import '../services/ble_service.dart';

class SettingsProvider extends ChangeNotifier {
  Settings _settings = Settings();
  final BleService _bleService = BleService();
  StreamSubscription? _bleSubscription;
  bool _isConnected = false;

  Settings get settings => _settings;
  bool get isConnected => _isConnected;

  SettingsProvider() {
    _bleSubscription = _bleService.settingsStream.listen((settings) {
      _settings = settings;
      notifyListeners();
    });
  }

  Future<void> connect() async {
    _isConnected = await _bleService.connect();
    notifyListeners();
    if (_isConnected) {
      await _bleService.requestSettings();
    }
  }

  Future<void> disconnect() async {
    await _bleService.disconnect();
    _isConnected = false;
    notifyListeners();
  }

  void updateSettings(Settings newSettings) {
    _settings = newSettings;
    _bleService.sendSettings(newSettings);
    notifyListeners();
  }

  void updateColor(int color) {
    _settings = _settings.copyWith(color: color);
    _bleService.sendSettings(_settings);
    notifyListeners();
  }

  void updateBrightness(int brightness) {
    _settings = _settings.copyWith(brightness: brightness);
    _bleService.sendSettings(_settings);
    notifyListeners();
  }

  void updateEffect(int effect) {
    _settings = _settings.copyWith(effect: effect);
    _bleService.sendSettings(_settings);
    notifyListeners();
  }

  void updateEffectSpeed(int speed) {
    _settings = _settings.copyWith(effectSpeed: speed);
    _bleService.sendSettings(_settings);
    notifyListeners();
  }

  void updateLongPressDuration(int duration) {
    _settings = _settings.copyWith(longPressDuration: duration);
    _bleService.sendSettings(_settings);
    notifyListeners();
  }

  void updateSidewaysThresholdTime(int time) {
    _settings = _settings.copyWith(sidewaysThresholdTime: time);
    _bleService.sendSettings(_settings);
    notifyListeners();
  }

  void updateSidewaysAccelThreshold(double threshold) {
    _settings = _settings.copyWith(sidewaysAccelThreshold: threshold);
    _bleService.sendSettings(_settings);
    notifyListeners();
  }

  void updateAccelAutoShutdown(bool value) {
    _settings = _settings.copyWith(accelAutoShutdown: value);
    _bleService.sendSettings(_settings);
    notifyListeners();
  }

  void updateAccelAutoStartup(bool value) {
    _settings = _settings.copyWith(accelAutoStartup: value);
    _bleService.sendSettings(_settings);
    notifyListeners();
  }

  @override
  void dispose() {
    _bleSubscription?.cancel();
    _bleService.dispose();
    super.dispose();
  }
}
