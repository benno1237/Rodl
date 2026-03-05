import 'dart:math';
import 'dart:convert';
import 'package:flutter/foundation.dart';
import 'package:shared_preferences/shared_preferences.dart';
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

  static const String _prefsKey = 'saved_rides_v1';

  RidesProvider() {
    // load persisted rides
    Future.microtask(() => _loadFromPrefs());
  }

  void loadMockData() {
    _rides = _generateMockRides();
    notifyListeners();
  }

  Future<void> addRideFromCsv(String csvContent) async {
    final ride = await CsvParser.parseRideAsync(_rides.length, csvContent);
    _rides.add(ride);
    await _saveToPrefs();
    notifyListeners();
  }

  Future<void> addRide(Ride ride) async {
    _rides.add(ride);
    await _saveToPrefs();
    notifyListeners();
  }

  Future<void> deleteRide(int id) async {
    _rides.removeWhere((r) => r.id == id);
    await _saveToPrefs();
    notifyListeners();
  }

  Future<void> renameRide(int id, String newName) async {
    final idx = _rides.indexWhere((r) => r.id == id);
    if (idx >= 0) {
      final r = _rides[idx];
      _rides[idx] = Ride(id: r.id, points: r.points, startTime: r.startTime, name: newName, sledId: r.sledId, username: r.username);
      await _saveToPrefs();
      notifyListeners();
    }
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

        // Assign a mock sled id for demo data (alternating)
        final mockSledId = (rides.length % 2 == 0) ? 'SLED-001' : 'SLED-002';
        rides.add(Ride(id: rides.length, points: points, startTime: rideStart, sledId: mockSledId));
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
          accelX: accel,
          accelY: 0,
          accelZ: 1,
        ),
      );

      timestamp += 100;
    }

    return points;
  }

  Future<void> _saveToPrefs() async {
    try {
      final prefs = await SharedPreferences.getInstance();
      final list = _rides.map((r) => r.toJson()).toList();
      final encoded = jsonEncode(list);
      await prefs.setString(_prefsKey, encoded);
    } catch (e) {
      if (kDebugMode) print('Failed to save rides: $e');
    }
  }

  Future<void> _loadFromPrefs() async {
    try {
      final prefs = await SharedPreferences.getInstance();
      final raw = prefs.getString(_prefsKey);
      if (raw == null || raw.isEmpty) return;
      final data = jsonDecode(raw) as List;
      _rides = data.map((e) => Ride.fromJson(Map<String, dynamic>.from(e))).toList();
      notifyListeners();
    } catch (e) {
      if (kDebugMode) print('Failed to load rides: $e');
    }
  }
}
