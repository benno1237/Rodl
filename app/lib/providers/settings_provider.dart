import 'dart:async';
import 'package:flutter/foundation.dart';
import '../models/settings.dart';
import '../services/ble_service.dart';

class SettingsProvider extends ChangeNotifier {
  Settings _settings = Settings();
  final BleService _bleService = BleService();
  StreamSubscription? _bleSubscription;
  bool _isConnected = false;
  String? _selectedTileRegion;

  String? get selectedTileRegion => _selectedTileRegion;

  Settings get settings => _settings;
  bool get isConnected => _isConnected;

  SettingsProvider() {
    _bleSubscription = _bleService.settingsStream.listen((settings) {
      _settings = settings;
      notifyListeners();
    });
  }

  /// Set the selected bundled tile region. If [region] is non-null, extract
  /// matching tiles from the bundled `assets/images/tiles.zip` into the
  /// app support `map_tiles` directory so the map tile provider can use them.
  Future<void> setSelectedTileRegion(String? region) async {
    _selectedTileRegion = region;
    notifyListeners();

    if (region == null) return;
  }

  /// Extract only tiles for [region] at [zoom] into the app support `map_tiles`
  /// directory for a quick preview. This is best-effort and will overwrite any
  /// existing preview tiles.
  Future<void> previewExtractTiles(String region, int zoom) async {
    // No-op: preview uses network tiles; keep method for compatibility.
    // ignore: avoid_print
    print('previewExtractTiles: no-op (using network tiles) for $region at $zoom');
    return;
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

  /// Update the current effect by canonical name (UI uses names).
  ///
  /// This stores the internal index in `Settings` for persistence/UI, and
  /// sends a copy to the backend with the `externalId` for the effect.
  void updateEffect(String effectName) {
    // Store the canonical name for UI/persistence.
    _settings = _settings.copyWith(effect: effectName);

    // Sending to backend: `Settings.toBytes()` will translate the stored
    // name into the correct external ID, so send the settings directly.
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
