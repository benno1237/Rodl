import 'dart:async';
import 'package:flutter/foundation.dart';
import 'package:shared_preferences/shared_preferences.dart';
import '../models/settings.dart';
import '../services/ble_service.dart';
import '../services/name_cache.dart';

class SettingsProvider extends ChangeNotifier {
  Settings _settings = Settings();
  String _username = '';
  bool _mockRidesEnabled = false;
  bool _mockLeaderboardEnabled = true;
  final BleService _bleService = BleService();
  StreamSubscription? _bleSubscription;
  StreamSubscription? _connectionSubscription;
  bool _isConnected = false;
  bool _isConnecting = false;
  Future<bool>? _pendingConnect;
  String? _selectedTileRegion;

  // ---- BLE device ↔ sled pairing ------------------------------------------
  // Maps a sled id (a cosmetic skin in the app) to the BLE remote id (MAC on
  // Android, CoreBluetooth UUID on iOS) of the physical board flashed for it.
  static const String _activeSledKey = 'active_sled_id';
  String? _activeSledId;
  final Map<String, String> _pairedAddresses = {};

  String? get selectedTileRegion => _selectedTileRegion;

  Settings get settings => _settings;
  bool get isConnected => _isConnected;
  bool get isConnecting => _isConnecting;
  String get username => _username;
  BleService get bleService => _bleService;

  /// The sled the current BLE link was established for, if any.
  String? get activeSledId => _activeSledId;

  SettingsProvider() {
    _bleSubscription = _bleService.settingsStream.listen((settings) {
      _settings = settings;
      notifyListeners();
    });
    _connectionSubscription = _bleService.connectionStream.listen((connected) {
      final changed = _isConnected != connected || _isConnecting;
      _isConnected = connected;
      if (connected) {
        _isConnecting = false;
        _bleService.requestSettings();
      } else {
        _activeSledId = null;
      }
      if (changed) notifyListeners();
    });
    _loadUsername();
    _loadMockFlags();
    _loadPairing();
  }

  bool get mockRidesEnabled => _mockRidesEnabled;
  bool get mockLeaderboardEnabled => _mockLeaderboardEnabled;

  Future<void> _loadMockFlags() async {
    try {
      final prefs = await SharedPreferences.getInstance();
      _mockRidesEnabled = prefs.getBool('mock_rides_enabled') ?? false;
      _mockLeaderboardEnabled = prefs.getBool('mock_leaderboard_enabled') ?? true;
      notifyListeners();
    } catch (_) {}
  }

  Future<void> setMockRidesEnabled(bool v) async {
    _mockRidesEnabled = v;
    try {
      final prefs = await SharedPreferences.getInstance();
      await prefs.setBool('mock_rides_enabled', v);
    } catch (_) {}
    notifyListeners();
  }

  Future<void> setMockLeaderboardEnabled(bool v) async {
    _mockLeaderboardEnabled = v;
    try {
      final prefs = await SharedPreferences.getInstance();
      await prefs.setBool('mock_leaderboard_enabled', v);
    } catch (_) {}
    notifyListeners();
  }

  /// Set the selected bundled tile region. If [region] is non-null, extract
  /// matching tiles from the bundled `assets/images/tiles.zip` into the
  /// app support `map_tiles` directory so the map tile provider can use them.
  Future<void> setSelectedTileRegion(String? region) async {
    _selectedTileRegion = region;
    notifyListeners();

    if (region == null) return;
  }

  // ---- BLE pairing helpers -------------------------------------------------

  Future<void> _loadPairing() async {
    try {
      final prefs = await SharedPreferences.getInstance();
      _activeSledId = prefs.getString(_activeSledKey);
      for (final key in prefs.getKeys()) {
        if (key.startsWith(_blePairPrefix)) {
          final value = prefs.getString(key);
          if (value != null && value.isNotEmpty) {
            _pairedAddresses[key.substring(_blePairPrefix.length)] = value;
          }
        }
      }
    } catch (_) {}
  }

  static const String _blePairPrefix = 'ble_pair_';

  /// The BLE remote id currently bound to [sledId], if one has been paired.
  String? pairedAddressFor(String sledId) => _pairedAddresses[sledId];

  /// Best display name for [sledId]: the live advertised name while we are
  /// connected to it, otherwise the last name seen for its paired board.
  /// Null when unknown - callers fall back to the JSON catalog name.
  String? sledDisplayName(String sledId) {
    if (_isConnected && _activeSledId == sledId) {
      final live = _bleService.remoteName;
      if (live != null && live.isNotEmpty) return live;
    }
    final mac = _pairedAddresses[sledId];
    if (mac != null) {
      NameCache.ensureLoaded(); // races startup; first call may miss
      final cached = NameCache.nameFor(mac);
      if (cached != null && cached.isNotEmpty) return cached;
    }
    return null;
  }

  /// The sled id bound to [mac], if any (inverse of [pairedAddressFor]).
  String? sledIdForMac(String mac) {
    for (final entry in _pairedAddresses.entries) {
      if (entry.value == mac) return entry.key;
    }
    return null;
  }

  /// All MACs currently bound to a sled (for discovery classification).
  Set<String> get pairedMacs => _pairedAddresses.values.toSet();

  Future<void> _savePairing(String sledId, String address) async {
    _pairedAddresses[sledId] = address;
    try {
      final prefs = await SharedPreferences.getInstance();
      await prefs.setString('$_blePairPrefix$sledId', address);
    } catch (_) {}
  }

  /// Drop the stored binding for [sledId]. If the active link belongs to this
  /// sled it is disconnected as well.
  Future<void> forgetSled(String sledId) async {
    _pairedAddresses.remove(sledId);
    try {
      final prefs = await SharedPreferences.getInstance();
      await prefs.remove('$_blePairPrefix$sledId');
    } catch (_) {}
    if (_activeSledId == sledId) {
      await disconnect();
    }
    notifyListeners();
  }

  /// Persist the sled the current link is for so the app can auto-reconnect
  /// to the same board on the next launch.
  Future<void> _setActiveSled(String? sledId) async {
    _activeSledId = sledId;
    try {
      final prefs = await SharedPreferences.getInstance();
      if (sledId == null) {
        await prefs.remove(_activeSledKey);
      } else {
        await prefs.setString(_activeSledKey, sledId);
      }
    } catch (_) {}
  }

  /// Connect to the physical board bound to [sledId].
  ///
  /// With [address] the connection goes straight to that board (used by the
  /// home screen, which already knows the address from a discovery scan) and
  /// the address is saved as this sled's pairing. Otherwise the stored
  /// pairing is used to reconnect directly; on first use (or if the stored
  /// address is stale) it scans and then binds the found board to this
  /// sled id.
  Future<bool> connectToSled(String sledId, {String? address}) async {
    // Already talking to this exact board?
    if (_isConnected && _activeSledId == sledId) {
      final current = _bleService.remoteId;
      if (address == null) {
        final paired = pairedAddressFor(sledId);
        if (paired == null || current == null || paired == current) {
          return true;
        }
        await disconnect(); // pairing points at a different board
      } else if (address == current) {
        return true;
      } else {
        await disconnect(); // switch to the requested board
      }
    } else if (_isConnected) {
      // Connected to some other sled — switch over.
      await disconnect();
    }
    return _pendingConnect ??= _connectToSled(sledId, address: address);
  }

  Future<bool> _connectToSled(String sledId, {String? address}) async {
    try {
      _isConnecting = true;
      notifyListeners();
      _isConnected = await _bleService.connect(
        targetAddress: address ?? pairedAddressFor(sledId),
      );
      if (_isConnected) {
        final usedAddress = address ?? _bleService.remoteId;
        if (usedAddress != null) await _savePairing(sledId, usedAddress);
        await _setActiveSled(sledId);
      } else {
        await _setActiveSled(null);
      }
      return _isConnected;
    } finally {
      _isConnecting = false;
      _pendingConnect = null;
      notifyListeners();
    }
  }

  /// Ensure the link is up for [sledId]; reconnects (and re-pairs if needed)
  /// when it is not.
  Future<bool> ensureConnectedToSled(String sledId) async {
    if (_isConnected && _activeSledId == sledId) return true;
    return connectToSled(sledId);
  }

  /// Generic connect with no sled context (used by the Settings screen).
  /// Scans for any board; does not create a pairing.
  Future<bool> connect() {
    if (_isConnected) return Future.value(true);
    return _pendingConnect ??= _connect();
  }

  Future<bool> _connect() async {
    try {
      _isConnecting = true;
      notifyListeners();
      _isConnected = await _bleService.connect();
      return _isConnected;
    } finally {
      _isConnecting = false;
      _pendingConnect = null;
      notifyListeners();
    }
  }

  Future<void> _loadUsername() async {
    try {
      final prefs = await SharedPreferences.getInstance();
      _username = prefs.getString('username') ?? '';
      notifyListeners();
    } catch (_) {}
  }

  Future<void> setUsername(String value) async {
    _username = value;
    try {
      final prefs = await SharedPreferences.getInstance();
      await prefs.setString('username', value);
    } catch (_) {}
    notifyListeners();
  }

  Future<void> disconnect() async {
    _pendingConnect = null;
    await _bleService.disconnect();
    _isConnected = false;
    _isConnecting = false;
    await _setActiveSled(null);
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
    _connectionSubscription?.cancel();
    _bleService.dispose();
    super.dispose();
  }
}
