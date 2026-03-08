import 'dart:convert';
import 'dart:math' as math;
import 'package:flutter/foundation.dart';
import 'package:flutter/services.dart' show rootBundle;
import 'package:xml/xml.dart' as xml;
import '../models/ride.dart';
import '../models/gps_point.dart';

class _TrackDef {
  final String id;
  final String name;
  final String gpxPath;
  final List<List<double>> points; // [lat, lon]
  final List<double>? centroid; // [lat, lon]

  _TrackDef({
    required this.id,
    required this.name,
    required this.gpxPath,
    required this.points,
    required this.centroid,
  });
}

class TrackMatchResult {
  final String trackId;
  final String trackName;
  final List<List<GPSPoint>> segments;

  TrackMatchResult({
    required this.trackId,
    required this.trackName,
    required this.segments,
  });
}

class TrackMatcher {
  static final TrackMatcher _instance = TrackMatcher._internal();

  factory TrackMatcher() => _instance;

  TrackMatcher._internal();

  final List<_TrackDef> _tracks = [];
  bool _loaded = false;

  Future<void> _loadTracks() async {
    if (_loaded) return;
    try {
      final jsonText = await rootBundle.loadString('assets/data/tracks.json');
      final List data = json.decode(jsonText) as List;
      for (final item in data) {
        final map = Map<String, dynamic>.from(item);
        final id = map['id'] as String? ?? map['name'] as String? ?? '';
        final name = map['name'] as String? ?? id;
        final gpxPath = map['gpxPath'] as String? ?? '';
        final pts = await _loadGpxPointsSafe(gpxPath);
        final centroid = _computeCentroidFromPoints(pts);
        _tracks.add(
          _TrackDef(
            id: id,
            name: name,
            gpxPath: gpxPath,
            points: pts,
            centroid: centroid,
          ),
        );
      }
    } catch (e, st) {
      debugPrint('TrackMatcher._loadTracks: failed to load tracks.json: $e');
      if (kDebugMode) debugPrint(st.toString());
    }
    _loaded = true;
    debugPrint('TrackMatcher: loaded ${_tracks.length} tracks');
  }

  Future<List<List<double>>> _loadGpxPointsSafe(String path) async {
    try {
      final txt = await rootBundle.loadString(path);
      return _parseGpxPoints(txt);
    } catch (_) {
      debugPrint('TrackMatcher: failed to load GPX asset at $path');
      return [];
    }
  }

  List<List<double>> _parseGpxPoints(String xmlText) {
    try {
      final doc = xml.XmlDocument.parse(xmlText);
      final trkpts = doc.findAllElements('trkpt');
      final pts = <List<double>>[];
      for (final e in trkpts) {
        final latAttr = e.getAttribute('lat');
        final lonAttr = e.getAttribute('lon');
        if (latAttr != null && lonAttr != null) {
          final lat = double.tryParse(latAttr);
          final lon = double.tryParse(lonAttr);
          if (lat != null && lon != null) pts.add([lat, lon]);
        }
      }
      return pts;
    } catch (e, st) {
      debugPrint('TrackMatcher._parseGpxPoints: parse error: $e');
      if (kDebugMode) debugPrint(st.toString());
      return [];
    }
  }

  List<double>? _computeCentroidFromPoints(List<List<double>> points) {
    if (points.isEmpty) return null;
    double lat = 0;
    double lon = 0;
    for (final p in points) {
      lat += p[0];
      lon += p[1];
    }
    return [lat / points.length, lon / points.length];
  }

  double _haversineMeters(double lat1, double lon1, double lat2, double lon2) {
    const earth = 6371000.0; // meters
    final dLat = _degToRad(lat2 - lat1);
    final dLon = _degToRad(lon2 - lon1);
    final a =
        math.sin(dLat / 2) * math.sin(dLat / 2) +
        math.cos(_degToRad(lat1)) *
            math.cos(_degToRad(lat2)) *
            math.sin(dLon / 2) *
            math.sin(dLon / 2);
    final c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a));
    return earth * c;
  }

  double _degToRad(double d) => d * math.pi / 180.0;

  /// Match ride to the best track (by centroid) and optionally split into
  /// segments (laps) using the track's start point as lap delimiter.
  /// Returns null if no suitable match is found.
  Future<TrackMatchResult?> matchAndSplit(
    Ride ride, {
    double centroidThresholdMeters = 1000.0,
    double distanceThresholdMeters = 25.0,
    int minSegmentPoints = 50,  // to avoid spurious segments from GPS noise, require at least this many points between passes
  }) async {
    if (ride.points.isEmpty) return null;
    await _loadTracks();
    if (_tracks.isEmpty) return null;

    // Compute ride centroid
    double rLat = 0, rLon = 0;
    for (final p in ride.points) {
      rLat += p.lat;
      rLon += p.lon;
    }
    rLat /= ride.points.length;
    rLon /= ride.points.length;

    // Stage 1: Choose track whose centroid is nearest to ride centroid and within threshold
    double bestCentroidDist = double.infinity;
    _TrackDef? bestTrack;
    debugPrint(
      'TrackMatcher: ride centroid=($rLat,$rLon) points=${ride.points.length}',
    );
    for (final t in _tracks) {
      if (t.points.isEmpty || t.centroid == null) continue;
      final tLat = t.centroid![0];
      final tLon = t.centroid![1];
      final d = _haversineMeters(rLat, rLon, tLat, tLon);
      debugPrint(
        'TrackMatcher: track ${t.id} centroid=($tLat,$tLon) dist=${d.toStringAsFixed(1)}m points=${t.points.length}',
      );
      if (d > centroidThresholdMeters) continue;
      if (d < bestCentroidDist) {
        bestCentroidDist = d;
        bestTrack = t;
      }
    }

    if (bestTrack == null) return null;

    // Stage 2: Split by matching a start endpoint then its opposite endpoint
    final startPt = bestTrack.points.first;
    final endPt = bestTrack.points.last;

    final segments = <List<GPSPoint>>[];

    bool inSegment = false;
    int segStart = -1;
    bool expectEndIsEnd = true; // when true, start was near startPt and we expect endPt to finish

    int idx = 0;
    while (idx < ride.points.length) {
      final p = ride.points[idx];
      final dStart = _haversineMeters(p.lat, p.lon, startPt[0], startPt[1]);
      final dEnd = _haversineMeters(p.lat, p.lon, endPt[0], endPt[1]);

      if (!inSegment) {
        if (dStart <= distanceThresholdMeters) {
          inSegment = true;
          segStart = idx;
          expectEndIsEnd = true;
          // consume this index and continue
          idx++;
          continue;
        } else if (dEnd <= distanceThresholdMeters) {
          inSegment = true;
          segStart = idx;
          expectEndIsEnd = false;
          idx++;
          continue;
        } else {
          idx++;
          continue;
        }
      } else {
        // we're inside a candidate segment; look for the expected endpoint
        final reachedEnd = expectEndIsEnd ? (dEnd <= distanceThresholdMeters) : (dStart <= distanceThresholdMeters);
        if (reachedEnd) {
          if (idx - segStart >= minSegmentPoints) {
            segments.add(ride.points.sublist(segStart, idx));
          }
          inSegment = false;
          // ignore the following points until a new start is detected; jump forward a bit
          idx = idx + minSegmentPoints;
          continue;
        } else {
          idx++;
          continue;
        }
      }
    }

    // If we found no well-formed start->end segments, fall back to previous behavior
    if (segments.isEmpty) {
      segments.add(List<GPSPoint>.from(ride.points));
    }
    final lens = segments.map((s) => s.length).toList();
    debugPrint(
      'TrackMatcher: produced ${segments.length} segments lengths=$lens',
    );

    return TrackMatchResult(
      trackId: bestTrack.id,
      trackName: bestTrack.name,
      segments: segments,
    );
  }

  /// Return GPX point list (lat,lon) for a given track id, or empty if not found.
  Future<List<List<double>>> getTrackPointsById(String trackId) async {
    await _loadTracks();
    for (final t in _tracks) {
      if (t.id == trackId) return t.points;
    }
    return [];
  }
}
