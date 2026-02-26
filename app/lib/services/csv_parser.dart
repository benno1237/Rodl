import 'dart:math';
import 'package:csv/csv.dart';
import '../models/gps_point.dart';
import '../models/ride.dart';

class CsvParser {
  static Ride parseRide(int rideId, String csvContent) {
    final List<List<dynamic>> rows = const CsvToListConverter().convert(
      csvContent,
    );

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
          acceleration: _mockAcceleration(timestamp),
        ),
      );
    }

    final startTime = points.isNotEmpty
        ? DateTime.now().subtract(Duration(milliseconds: points.last.timestamp))
        : DateTime.now();

    return Ride(id: rideId, points: points, startTime: startTime);
  }

  static double _mockAcceleration(int timestamp) {
    final random = Random(timestamp);
    return 0.5 + random.nextDouble() * 1.0;
  }

  static String rideToCsv(Ride ride) {
    final List<List<dynamic>> rows = [
      ['timestamp', 'lat', 'lon', 'speed_kmh', 'alt_m', 'sats', 'hdop', 'age'],
      ...ride.points.map(
        (p) => [
          p.timestamp,
          p.lat.toStringAsFixed(7),
          p.lon.toStringAsFixed(7),
          p.speedKmh.toStringAsFixed(1),
          p.altM.toStringAsFixed(1),
          p.sats,
          p.hdop.toStringAsFixed(1),
          p.age,
        ],
      ),
    ];
    return const ListToCsvConverter().convert(rows);
  }
}
