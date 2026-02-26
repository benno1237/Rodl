import 'dart:math';
import 'package:flutter/foundation.dart';
import '../models/ride.dart';
import '../models/gps_point.dart';
import '../services/csv_parser.dart';

class RidesProvider extends ChangeNotifier {
  List<Ride> _rides = [];

  List<Ride> get rides => _rides;

  Map<DateTime, List<Ride>> get ridesByDay {
    final Map<DateTime, List<Ride>> grouped = {};
    for (var ride in _rides) {
      final date = ride.date;
      if (!grouped.containsKey(date)) {
        grouped[date] = [];
      }
      grouped[date]!.add(ride);
    }
    return grouped;
  }

  List<Ride> getRidesForDay(DateTime date) {
    final targetDate = DateTime(date.year, date.month, date.day);
    return _rides.where((r) => r.date == targetDate).toList();
  }

  void loadMockData() {
    _rides = _generateMockRides();
    notifyListeners();
  }

  void addRideFromCsv(String csvContent) {
    final ride = CsvParser.parseRide(_rides.length, csvContent);
    _rides.add(ride);
    notifyListeners();
  }

  List<Ride> _generateMockRides() {
    final random = Random(42);
    final List<Ride> rides = [];
    final now = DateTime.now();

    final mockLocations = [
      {'lat': 47.3769, 'lon': 8.5417, 'name': 'Zürich'},
      {'lat': 47.4500, 'lon': 8.5500, 'name': 'Zürichsee'},
      {'lat': 47.5000, 'lon': 8.7500, 'name': 'Winterthur'},
    ];

    for (int day = 0; day < 5; day++) {
      final ridesPerDay = random.nextInt(3) + 1;
      final baseDate = now.subtract(Duration(days: day));

      for (int r = 0; r < ridesPerDay; r++) {
        final location = mockLocations[random.nextInt(mockLocations.length)];
        final startHour = 8 + random.nextInt(12);
        final rideStart = baseDate.add(
          Duration(hours: startHour, minutes: random.nextInt(60)),
        );

        final points = _generateMockTrack(
          location['lat'] as double,
          location['lon'] as double,
          random,
        );

        rides.add(Ride(id: rides.length, points: points, startTime: rideStart));
      }
    }

    return rides;
  }

  List<GPSPoint> _generateMockTrack(
    double baseLat,
    double baseLon,
    Random random,
  ) {
    final List<GPSPoint> points = [];
    const int pointCount = 50;

    double lat = baseLat;
    double lon = baseLon;
    int timestamp = 0;

    for (int i = 0; i < pointCount; i++) {
      lat += (random.nextDouble() - 0.5) * 0.002;
      lon += (random.nextDouble() - 0.5) * 0.002;

      final speed = 10.0 + random.nextDouble() * 25.0;
      final accel = 0.5 + random.nextDouble() * 1.0;

      points.add(
        GPSPoint(
          timestamp: timestamp,
          lat: lat,
          lon: lon,
          speedKmh: speed,
          altM: 400 + random.nextDouble() * 100,
          sats: 8 + random.nextInt(4),
          hdop: 1.0 + random.nextDouble() * 2.0,
          age: 50 + random.nextInt(100),
          acceleration: accel,
        ),
      );

      timestamp += 100;
    }

    return points;
  }
}
