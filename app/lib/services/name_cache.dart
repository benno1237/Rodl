import 'package:shared_preferences/shared_preferences.dart';

/// Last-known display names of boards, keyed by BLE remote id (MAC).
///
/// The PCB is the source of truth for its name (it advertises it); this cache
/// only lets the home screen show the right name for boards that are currently
/// out of range.
class NameCache {
  static final Map<String, String> _names = {};
  static bool _loaded = false;

  static Future<void> ensureLoaded() async {
    if (_loaded) return;
    try {
      final prefs = await SharedPreferences.getInstance();
      for (final key in prefs.getKeys()) {
        if (key.startsWith('ble_name_')) {
          final value = prefs.getString(key);
          if (value != null && value.isNotEmpty) {
            _names[key.substring('ble_name_'.length)] = value;
          }
        }
      }
    } catch (_) {}
    _loaded = true;
  }

  static String? nameFor(String remoteId) => _names[remoteId];

  static Future<void> set(String remoteId, String name) async {
    final trimmed = name.trim();
    if (trimmed.isEmpty) return;
    _names[remoteId] = trimmed;
    try {
      final prefs = await SharedPreferences.getInstance();
      await prefs.setString('ble_name_$remoteId', trimmed);
    } catch (_) {}
  }
}
