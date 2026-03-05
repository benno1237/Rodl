import 'dart:math';
import 'dart:convert';
import 'package:flutter/foundation.dart';
import '../models/gps_point.dart';
import '../models/ride.dart';

/// Helper used by `compute` to parse CSV in a background isolate.
Map<String, dynamic> _parseRideEntry(Map args) {
  final int rideId = args['rideId'] as int;
  final String csvContent = args['csv'] as String;

  final List<List<dynamic>> rows = _simpleCsvToRows(csvContent);

  if (rows.isEmpty) {
    return {
      'rideId': rideId,
      'points': [],
      'startTimeMillis': DateTime.now().millisecondsSinceEpoch,
    };
  }

  final List<Map<String, dynamic>> points = [];
  final bool hasHeader = rows[0][0] == 'timestamp';

  for (int i = hasHeader ? 1 : 0; i < rows.length; i++) {
    final row = rows[i];
    if (row.length < 8) continue;

    final timestamp = int.tryParse(row[0].toString()) ?? 0;
    final lat = double.tryParse(row[1].toString()) ?? 0.0;
    final lon = double.tryParse(row[2].toString()) ?? 0.0;
    final speed = double.tryParse(row[3].toString()) ?? 0.0;
    final alt = double.tryParse(row[4].toString()) ?? 0.0;
    final sats = int.tryParse(row[5].toString()) ?? 0;
    final hdop = double.tryParse(row[6].toString()) ?? 99.9;
    final age = int.tryParse(row[7].toString()) ?? 0;

    points.add({
      'timestamp': timestamp,
      'lat': lat,
      'lon': lon,
      'speedKmh': speed,
      'altM': alt,
      'sats': sats,
      'hdop': hdop,
      'age': age,
      'acceleration': 0.5 + Random(timestamp).nextDouble() * 1.0,
    });
  }

  final startTimeMillis = points.isNotEmpty
      ? DateTime.now().subtract(Duration(milliseconds: points.last['timestamp'] as int)).millisecondsSinceEpoch
      : DateTime.now().millisecondsSinceEpoch;

  return {
    'rideId': rideId,
    'points': points,
    'startTimeMillis': startTimeMillis,
  };
}

List<List<dynamic>> _simpleCsvToRows(String csvContent) {
  final lines = const LineSplitter().convert(csvContent);
  final rows = <List<dynamic>>[];
  for (final line in lines) {
    if (line.trim().isEmpty) continue;
    rows.add(line.split(',').map((s) => s.trim()).toList());
  }
  return rows;
}

class CsvParser {
  static Ride parseRide(int rideId, String csvContent) {
    final List<List<dynamic>> rows = _simpleCsvToRows(csvContent);

    if (rows.isEmpty) {
      return Ride(id: rideId, points: [], startTime: DateTime.now());
    }

    final List<GPSPoint> points = [];
    final bool hasHeader = rows[0][0] == 'timestamp';

    for (int i = hasHeader ? 1 : 0; i < rows.length; i++) {
      final row = rows[i];
      if (row.length < 8) continue;

      final timestamp = int.tryParse(row[0].toString()) ?? 0;
      final lat = double.tryParse(row[1].toString()) ?? 0.0;
      final lon = double.tryParse(row[2].toString()) ?? 0.0;
      final speed = double.tryParse(row[3].toString()) ?? 0.0;
      final alt = double.tryParse(row[4].toString()) ?? 0.0;
      final sats = int.tryParse(row[5].toString()) ?? 0;
      final hdop = double.tryParse(row[6].toString()) ?? 99.9;
      final age = int.tryParse(row[7].toString()) ?? 0;

      points.add(
        GPSPoint(
          timestamp: timestamp,
          lat: lat,
          lon: lon,
          speedKmh: speed,
          altM: alt,
          sats: sats,
          hdop: hdop,
          age: age,
          accelX: _mockAcceleration(timestamp),
          accelY: 0,
          accelZ: 1,
        ),
      );
    }

    final startTime = points.isNotEmpty
        ? DateTime.now().subtract(Duration(milliseconds: points.last.timestamp))
        : DateTime.now();

    return Ride(id: rideId, points: points, startTime: startTime);
  }

  /// Async parsing that runs in a background isolate and reconstructs the Ride.
  static Future<Ride> parseRideAsync(int rideId, String csvContent) async {
    final Map<String, dynamic> result = await compute(_parseRideEntry, {
      'rideId': rideId,
      'csv': csvContent,
    });

    final List<dynamic> pointsData = result['points'] as List<dynamic>? ?? [];
    final List<GPSPoint> points = pointsData.map((p) {
      return GPSPoint(
        timestamp: p['timestamp'] as int,
        lat: (p['lat'] as num).toDouble(),
        lon: (p['lon'] as num).toDouble(),
        speedKmh: (p['speedKmh'] as num).toDouble(),
        altM: (p['altM'] as num).toDouble(),
        sats: p['sats'] as int,
        hdop: (p['hdop'] as num).toDouble(),
        age: p['age'] as int,
        accelX: p.containsKey('accelX') ? (p['accelX'] as num).toDouble() : 0,
        accelY: p.containsKey('accelY') ? (p['accelY'] as num).toDouble() : 0,
        accelZ: p.containsKey('accelZ') ? (p['accelZ'] as num).toDouble() : 1,
      );
    }).toList();

    final startTimeMillis = result['startTimeMillis'] as int?;
    final startTime = startTimeMillis != null
        ? DateTime.fromMillisecondsSinceEpoch(startTimeMillis)
        : DateTime.now();

    return Ride(id: rideId, points: points, startTime: startTime);
  }

  static double _mockAcceleration(int timestamp) {
    final random = Random(timestamp);
    return 0.5 + random.nextDouble() * 1.0;
  }
}
