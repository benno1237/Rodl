import 'package:flutter/material.dart';

class Settings {
  final int color;
  final int brightness;
  final int effect;
  final int effectSpeed;
  final int longPressDuration;
  final int sidewaysThresholdTime;
  final double sidewaysAccelThreshold;
  final bool accelAutoShutdown;
  final bool accelAutoStartup;

  Settings({
    this.color = 0xFF7530AA,
    this.brightness = 150,
    this.effect = 0,
    this.effectSpeed = 500,
    this.longPressDuration = 1500,
    this.sidewaysThresholdTime = 1000,
    this.sidewaysAccelThreshold = 0.7,
    this.accelAutoShutdown = true,
    this.accelAutoStartup = false,
  });

  Color get colorAsColor => Color(color);

  Settings copyWith({
    int? color,
    int? brightness,
    int? effect,
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
      effect,
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
    return Settings(
      color: color,
      brightness: bytes[4],
      effect: bytes[5],
      effectSpeed: (bytes[6] << 8) | bytes[7],
      longPressDuration: (bytes[8] << 8) | bytes[9],
      sidewaysThresholdTime: (bytes[10] << 8) | bytes[11],
      sidewaysAccelThreshold: bytes[12] / 10.0,
      accelAutoShutdown: bytes[13] == 1,
      accelAutoStartup: bytes[14] == 1,
    );
  }
}
