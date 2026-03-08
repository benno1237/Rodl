import 'dart:convert';
import 'package:flutter/services.dart' show rootBundle;
import 'package:flutter/foundation.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'dart:io';
import 'package:path_provider/path_provider.dart';
import '../models/ride.dart';
import '../services/track_matcher.dart';
import '../models/gps_point.dart';
import '../services/csv_parser.dart';
import '../services/gpx_loader.dart';

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

  /// Populate mock/demo rides on demand. These rides are marked with
  /// `isMock == true` so they can be removed later when the user disables
  /// mock rides.
  void loadMockData() {
    final now = DateTime.now();
    final List<Ride> mock = [];
    for (int i = 0; i < 5; i++) {
      final start = now.subtract(Duration(days: i));
      final points = <GPSPoint>[];
      for (int j = 0; j < 40; j++) {
        points.add(GPSPoint(
          timestamp: j * 100,
          lat: 47.37 + (i + j) * 0.0001,
          lon: 8.54 + (i + j) * 0.0001,
          speedKmh: 20 + (j % 10),
          altM: 400.0,
          sats: 8,
          hdop: 1.0,
          age: 0,
        ));
      }
      mock.add(Ride(id: _rides.length + mock.length, points: points, startTime: start, isMock: true));
    }
    _rides.addAll(mock);
    notifyListeners();
    _saveToPrefs();
  }

  /// Remove all previously generated mock rides.
  Future<void> removeMockRides() async {
    _rides.removeWhere((r) => r.isMock == true);
    await _saveToPrefs();
    notifyListeners();
  }

  /// Analyze all stored rides and run track matching/splitting on each recorded
  /// ride. Mock rides are skipped. This replaces the in-memory list with the
  /// processed results and persists them.
  Future<void> analyzeAndMatchAll({double distanceThresholdMeters = 50.0}) async {
    debugPrint('RidesProvider: analyzeAndMatchAll started, total stored rides=${_rides.length}');
    if (_rides.isEmpty) {
      try {
        final prefs = await SharedPreferences.getInstance();
        final raw = prefs.getString(_prefsKey);
        debugPrint('RidesProvider: direct prefs read raw=${raw == null ? 'null' : 'length=${raw.length}'}');
      } catch (e) {
        debugPrint('RidesProvider: direct prefs read failed: $e');
      }
    }
    final List<Ride> newRides = [];
    int idx = 0;
    for (final ride in List<Ride>.from(_rides)) {
      debugPrint('RidesProvider: processing index=$idx id=${ride.id} isMock=${ride.isMock} points=${ride.points.length}');
      idx++;
      if (ride.isMock) {
        debugPrint('RidesProvider: skipping mock ride id=${ride.id}');
        newRides.add(ride);
        continue;
      }
      try {
        final result = await TrackMatcher().matchAndSplit(ride, distanceThresholdMeters: distanceThresholdMeters);
        if (result == null) {
          debugPrint('RidesProvider: ride id=${ride.id} not matched to any track');
          newRides.add(ride);
        } else {
          debugPrint('RidesProvider: ride id=${ride.id} matched to ${result.trackName} (id=${result.trackId}) segments=${result.segments.length}');
          if (result.segments.length <= 1) {
            final enriched = Ride(
              id: newRides.length,
              points: ride.points,
              startTime: ride.startTime,
              name: ride.name,
              sledId: ride.sledId,
              username: ride.username,
              trackName: result.trackName,
              trackId: result.trackId,
              isMock: ride.isMock,
            );
            newRides.add(enriched);
          } else {
            for (final seg in result.segments) {
              final segRide = Ride(
                id: newRides.length,
                points: seg,
                startTime: DateTime.fromMillisecondsSinceEpoch(seg.first.timestamp),
                name: ride.name,
                sledId: ride.sledId,
                username: ride.username,
                trackName: result.trackName,
                trackId: result.trackId,
                isMock: ride.isMock,
              );
              debugPrint('RidesProvider: adding segment ride id=${segRide.id} points=${segRide.points.length}');
              newRides.add(segRide);
            }
          }
        }
      } catch (e, st) {
        debugPrint('RidesProvider.analyzeAndMatchAll: failed for ride ${ride.id}: $e');
        if (kDebugMode) debugPrint(st.toString());
        newRides.add(ride);
      }
    }
    _rides = newRides;
    await _saveToPrefs();
    debugPrint('RidesProvider: analyzeAndMatchAll complete, resulting rides=${_rides.length}');
    notifyListeners();
  }

  /// Insert a few sample recorded rides based on available GPX tracks.
  /// These are marked as real (isMock=false) so they are considered for analysis.
  Future<void> loadSampleRecordedRides({int lapsPerSample = 1}) async {
    try {
      // load the tracks list from TrackMatcher
      // pick first two tracks if available
      final prefsList = <Ride>[];
      // we need to access tracks; read tracks.json to get ids
      final tm = TrackMatcher();
      // try to read tracks.json directly to get track ids (preserve order)
      final jsonText = await rootBundle.loadString('assets/data/tracks.json');
      final List data = json.decode(jsonText) as List;
      int created = 0;
      for (final item in data) {
        if (created >= 2) break;
        final map = Map<String, dynamic>.from(item);
        final id = map['id'] as String? ?? map['name'] as String? ?? '';
        final points = await tm.getTrackPointsById(id);
        if (points.isEmpty) continue;

        // create one sample ride per track (possibly multiple laps)
        final now = DateTime.now().subtract(Duration(days: created));
        final gpsPoints = <GPSPoint>[];
        int ts = now.millisecondsSinceEpoch;
        for (int lap = 0; lap < lapsPerSample; lap++) {
          for (final pt in points) {
            // add small noise
            final lat = pt[0] + ((created + lap) % 3 - 1) * 0.00001;
            final lon = pt[1] + ((created + lap) % 5 - 2) * 0.00001;
            gpsPoints.add(GPSPoint(
              timestamp: ts,
              lat: lat,
              lon: lon,
              speedKmh: 25.0,
              altM: 400.0,
              sats: 8,
              hdop: 1.0,
              age: 0,
            ));
            ts += 1000; // 1s per point
          }
        }

        final ride = Ride(id: _rides.length + prefsList.length, points: gpsPoints, startTime: DateTime.fromMillisecondsSinceEpoch(gpsPoints.first.timestamp), isMock: false);
        prefsList.add(ride);
        created++;
      }

      if (prefsList.isNotEmpty) {
        _rides.addAll(prefsList);
        await _saveToPrefs();
        notifyListeners();
      }
    } catch (e, st) {
      debugPrint('RidesProvider.loadSampleRecordedRides failed: $e');
      if (kDebugMode) debugPrint(st.toString());
    }
  }

  /// Import GPX files placed in the app external files directory
  /// (e.g., Android: /sdcard/Android/data/{package}/files). Parses each
  /// `.gpx` file and adds it as a recorded ride.
  Future<int> importGpxFromAppFiles() async {
    try {
      final dir = await getExternalStorageDirectory();
      if (dir == null) {
        debugPrint('RidesProvider.importGpxFromAppFiles: external storage dir not available');
        return 0;
      }
      debugPrint('RidesProvider.importGpxFromAppFiles: scanning ${dir.path}');
      final files = Directory(dir.path).listSync();
      int added = 0;
      for (final f in files) {
        if (f is File && f.path.toLowerCase().endsWith('.gpx')) {
          try {
            final content = await f.readAsString();
            final pts = GpxLoader.parseGpxString(content);
            if (pts.isEmpty) {
              debugPrint('RidesProvider: GPX ${f.path} parsed 0 points, skipping');
              continue;
            }
            final ride = Ride(
              id: _rides.length + added,
              points: pts,
              startTime: DateTime.fromMillisecondsSinceEpoch(pts.first.timestamp),
              isMock: false,
            );
            _rides.add(ride);
            added++;
            debugPrint('RidesProvider: imported ${f.path} as ride id=${ride.id} points=${pts.length}');
          } catch (e, st) {
            debugPrint('RidesProvider: failed to import ${f.path}: $e');
            if (kDebugMode) debugPrint(st.toString());
          }
        }
      }
      if (added > 0) {
        await _saveToPrefs();
        notifyListeners();
      }
      return added;
    } catch (e, st) {
      debugPrint('RidesProvider.importGpxFromAppFiles: error: $e');
      if (kDebugMode) debugPrint(st.toString());
      return 0;
    }
  }

  Future<void> addRideFromCsv(String csvContent) async {
    final ride = await CsvParser.parseRideAsync(_rides.length, csvContent);
    // attempt to match to known tracks and split into laps when applicable
    try {
      final result = await TrackMatcher().matchAndSplit(ride);
      if (result == null) {
        _rides.add(ride);
      } else {
        // if multiple segments, add each as its own Ride
        if (result.segments.length <= 1) {
          final enriched = Ride(
            id: ride.id,
            points: ride.points,
            startTime: ride.startTime,
            name: ride.name,
            sledId: ride.sledId,
            username: ride.username,
            trackName: result.trackName,
            trackId: result.trackId,
          );
          _rides.add(enriched);
        } else {
          for (final seg in result.segments) {
            final segRide = Ride(
              id: _rides.length,
              points: seg,
              startTime: DateTime.fromMillisecondsSinceEpoch(seg.first.timestamp),
              name: ride.name,
              sledId: ride.sledId,
              username: ride.username,
              trackName: result.trackName,
              trackId: result.trackId,
            );
            _rides.add(segRide);
          }
        }
      }
    } catch (_) {
      _rides.add(ride);
    }
    await _saveToPrefs();
    notifyListeners();
  }

  Future<void> addRide(Ride ride) async {
    // if ride already has a track label, keep it; otherwise attempt to match and split
    try {
      final result = await TrackMatcher().matchAndSplit(ride);
      if (result == null) {
        _rides.add(ride);
      } else {
        if (result.segments.length <= 1) {
          final enriched = Ride(
            id: ride.id,
            points: ride.points,
            startTime: ride.startTime,
            name: ride.name,
            sledId: ride.sledId,
            username: ride.username,
            trackName: result.trackName,
            trackId: result.trackId,
          );
          _rides.add(enriched);
        } else {
          for (final seg in result.segments) {
            final segRide = Ride(
              id: _rides.length,
              points: seg,
              startTime: DateTime.fromMillisecondsSinceEpoch(seg.first.timestamp),
              name: ride.name,
              sledId: ride.sledId,
              username: ride.username,
              trackName: result.trackName,
              trackId: result.trackId,
            );
            _rides.add(segRide);
          }
        }
      }
    } catch (_) {
      _rides.add(ride);
    }
    await _saveToPrefs();
    notifyListeners();
  }

  Future<void> deleteRide(int id) async {
    _rides.removeWhere((r) => r.id == id);
    await _saveToPrefs();
    notifyListeners();
  }

  /// Remove all stored rides (permanently).
  Future<void> removeAllRides() async {
    _rides.clear();
    await _saveToPrefs();
    notifyListeners();
  }

  Future<void> renameRide(int id, String newName) async {
    final idx = _rides.indexWhere((r) => r.id == id);
    if (idx >= 0) {
      final r = _rides[idx];
      _rides[idx] = Ride(
        id: r.id,
        points: r.points,
        startTime: r.startTime,
        name: newName,
        sledId: r.sledId,
        username: r.username,
        trackId: r.trackId,
        trackName: r.trackName,
      );
      await _saveToPrefs();
      notifyListeners();
    }
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
      if (raw == null || raw.isEmpty) {
        debugPrint('RidesProvider._loadFromPrefs: no stored rides under key=$_prefsKey');
        return;
      }
      debugPrint('RidesProvider._loadFromPrefs: raw stored rides length=${raw.length}');
      final preview = raw.length > 400 ? '${raw.substring(0, 400)}...': raw;
      debugPrint('RidesProvider._loadFromPrefs preview: $preview');
      final data = jsonDecode(raw) as List;
      _rides = data.map((e) => Ride.fromJson(Map<String, dynamic>.from(e))).toList();
      notifyListeners();
    } catch (e) {
      debugPrint('RidesProvider._loadFromPrefs: failed to load rides: $e');
      if (kDebugMode) debugPrint(e.toString());
    }
  }
}
