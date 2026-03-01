import 'dart:collection';
import 'package:flutter/material.dart';

class EffectSpec {
  final int externalId;
  final bool segmented;
  final bool singleColor;
  final String? displayName;
  final int defaultSpeed;
  final Duration? durationOverride;

  const EffectSpec({
    required this.externalId,
    this.segmented = false,
    this.singleColor = false,
    this.displayName,
    this.defaultSpeed = 500,
    this.durationOverride,
  });
}

class Settings {
  final int color;
  final int brightness;
  final String effect; // stored as canonical name for persistence
  final int effectSpeed;
  final int longPressDuration;
  final int sidewaysThresholdTime;
  final double sidewaysAccelThreshold;
  final bool accelAutoShutdown;
  final bool accelAutoStartup;

  Settings({
    this.color = 0xFF7530AA,
    this.brightness = 150,
    this.effect = 'strobe',
    this.effectSpeed = 500,
    this.longPressDuration = 1500,
    this.sidewaysThresholdTime = 1000,
    this.sidewaysAccelThreshold = 0.7,
    this.accelAutoShutdown = true,
    this.accelAutoStartup = false,
  });

  /// Single source-of-truth describing effects. Keys are canonical effect
  /// names; values are metadata. The insertion order defines the UI ordering.
  static final Map<String, EffectSpec> effects = LinkedHashMap.fromEntries([
    MapEntry('static', EffectSpec(externalId: 0, segmented: false, singleColor: true, displayName: 'Static', defaultSpeed: 1000)),
    MapEntry('breathing', EffectSpec(externalId: 1, segmented: false, singleColor: true, displayName: 'Breathing', defaultSpeed: 2000)),
    MapEntry('rainbow', EffectSpec(externalId: 2, segmented: false, singleColor: false, displayName: 'Rainbow', defaultSpeed: 3000)),
    MapEntry('police', EffectSpec(externalId: 3, segmented: false, singleColor: true, displayName: 'Police', defaultSpeed: 600)),
    MapEntry('strobe', EffectSpec(externalId: 4, segmented: false, singleColor: true, displayName: 'Strobe', defaultSpeed: 200)),
    MapEntry('random', EffectSpec(externalId: 5, segmented: true, singleColor: false, displayName: 'Random', defaultSpeed: 700)),
    MapEntry('racing', EffectSpec(externalId: 6, segmented: true, singleColor: true, displayName: 'Racing', defaultSpeed: 1500)),
    MapEntry('rainbow_wipe', EffectSpec(externalId: 7, segmented: true, singleColor: false, displayName: 'Rainbow Wipe', defaultSpeed: 2500)),
    MapEntry('static_wipe', EffectSpec(externalId: 8, segmented: true, singleColor: true, displayName: 'Static Wipe', defaultSpeed: 2500)),
  ]);

  /// Return the per-effect default speed (same units as `Settings.effectSpeed`).
  static int getDefaultSpeedForName(String name) => effects[name]?.defaultSpeed ?? 500;

  /// Get canonical effect name (stored directly). Falls back to first entry.
  String get effectName {
    if (effects.containsKey(effect)) return effect;
    return Settings.effectNames.first;
  }

  /// Convert an effect name to its canonical index (based on insertion order).
  /// Returns 0 for unknown names.
  static int effectIndexFromName(String name) {
    final lower = name.toLowerCase();
    int idx = 0;
    for (final key in effects.keys) {
      if (key.toLowerCase() == lower) return idx;
      idx++;
    }
    return 0;
  }

  /// Human-friendly display names for effects (capitalized).
  static List<String> get effectDisplayNames {
    final entries = effects.entries.toList();
    return entries.map((e) {
      if (e.value.displayName != null) return e.value.displayName!;
      final parts = e.key.split('_');
      return parts.map((p) => p.isEmpty ? p : (p[0].toUpperCase() + p.substring(1))).join(' ');
    }).toList();
  }

  /// Ordered list of canonical effect names (insertion order of `effects`).
  static List<String> get effectNames => effects.keys.toList();

  /// Helpers to translate between UI names and backend integer IDs.
  static int? getExternalIdForName(String name) => effects[name]?.externalId;

  static String? getNameForExternalId(int id) {
    for (final entry in effects.entries) {
      if (entry.value.externalId == id) return entry.key;
    }
    return null;
  }

  Color get colorAsColor => Color(color);

  Settings copyWith({
    int? color,
    int? brightness,
    String? effect,
    int? effectSpeed,
    int? longPressDuration,
    int? sidewaysThresholdTime,
    double? sidewaysAccelThreshold,
    bool? accelAutoShutdown,
    bool? accelAutoStartup,
  }) {
    return Settings(
      color: color ?? this.color,
      brightness: brightness ?? this.brightness,
      effect: effect ?? this.effect,
      effectSpeed: effectSpeed ?? this.effectSpeed,
      longPressDuration: longPressDuration ?? this.longPressDuration,
      sidewaysThresholdTime:
          sidewaysThresholdTime ?? this.sidewaysThresholdTime,
      sidewaysAccelThreshold:
          sidewaysAccelThreshold ?? this.sidewaysAccelThreshold,
      accelAutoShutdown: accelAutoShutdown ?? this.accelAutoShutdown,
      accelAutoStartup: accelAutoStartup ?? this.accelAutoStartup,
    );
  }

  List<int> toBytes() {
    return [
      (color >> 24) & 0xFF,
      (color >> 16) & 0xFF,
      (color >> 8) & 0xFF,
      color & 0xFF,
      brightness,
      // Map the stored effect name to an external integer ID for the backend.
      (getExternalIdForName(effect) ?? effectIndexFromName(effect)),
      (effectSpeed >> 8) & 0xFF,
      effectSpeed & 0xFF,
      (longPressDuration >> 8) & 0xFF,
      longPressDuration & 0xFF,
      (sidewaysThresholdTime >> 8) & 0xFF,
      sidewaysThresholdTime & 0xFF,
      (sidewaysAccelThreshold * 10).round(),
      accelAutoShutdown ? 1 : 0,
      accelAutoStartup ? 1 : 0,
    ];
  }

  factory Settings.fromBytes(List<int> bytes) {
    if (bytes.length < 15) {
      return Settings();
    }
    final color =
        (bytes[0] << 24) | (bytes[1] << 16) | (bytes[2] << 8) | bytes[3];
    final extId = bytes[5];
    final name = getNameForExternalId(extId) ?? effectNames.first;

    return Settings(
      color: color,
      brightness: bytes[4],
      effect: name,
      effectSpeed: (bytes[6] << 8) | bytes[7],
      longPressDuration: (bytes[8] << 8) | bytes[9],
      sidewaysThresholdTime: (bytes[10] << 8) | bytes[11],
      sidewaysAccelThreshold: bytes[12] / 10.0,
      accelAutoShutdown: bytes[13] == 1,
      accelAutoStartup: bytes[14] == 1,
    );
  }
}

