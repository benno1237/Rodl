import 'dart:async';
import 'dart:convert';

import 'package:flutter/foundation.dart';
import 'package:flutter/services.dart';
// NOTE: geolocator is used by the native/background implementation; not
// referenced directly in this scaffold file.
import 'package:shared_preferences/shared_preferences.dart';

import '../models/gps_point.dart';

// Key used to persist background-recorded GPS points while the service runs.
const _kBgPointsKey = 'bg_recording_points_v1';

// Entry point for the foreground-task. The package expects a top-level
// callback that registers a TaskHandler implementation.
// NOTE: This file contains a minimal, non-invasive scaffold for background
// recording. To implement a fully functioning Android foreground service you
// should integrate `flutter_foreground_task` and implement platform-specific
// notification options. The functions below provide a simple, testable
// contract that the UI can call; they persist recorded points and expose
// a trivial start/stop mechanism using a platform MethodChannel if desired.

const MethodChannel _channel = MethodChannel('rodl/foreground_service');

Future<void> startForegroundRecording({required String title, required String text}) async {
  // Best-effort: try to invoke a platform method to start a native foreground
  // service. If native side isn't implemented, we still return without error.
  try {
    await _channel.invokeMethod('start', {'title': title, 'text': text});
  } catch (e) {
    if (kDebugMode) print('startForegroundRecording: native start not available: $e');
  }
}

Future<void> stopForegroundRecording() async {
  try {
    await _channel.invokeMethod('stop');
  } catch (e) {
    if (kDebugMode) print('stopForegroundRecording: native stop not available: $e');
  }
}

Future<List<GPSPoint>> loadBackgroundRecordedPoints() async {
  final prefs = await SharedPreferences.getInstance();
  final raw = prefs.getString(_kBgPointsKey) ?? '[]';
  final List data = jsonDecode(raw) as List;
  await prefs.remove(_kBgPointsKey);
  return data.map((e) => GPSPoint.fromJson(Map<String, dynamic>.from(e as Map))).toList();
}
