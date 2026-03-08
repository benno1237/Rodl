import 'dart:convert';

import 'package:flutter/foundation.dart';
import 'package:flutter/services.dart';
import 'package:xml/xml.dart' as xml;

import '../models/track.dart';

class _TracksStore {
  static List<Track> baseTracks = [];
  static Map<String, Track> tracksById = {};

  static Future<void> init() async {
    try {
      final raw = await rootBundle.loadString('assets/data/tracks.json');
      final list = json.decode(raw) as List<dynamic>;
      final parsedTracks = list
          .map((e) => Track.fromJson(e as Map<String, dynamic>))
          .toList();

      // Compute missing centroids once at init so runtime code can reuse them.
      baseTracks = await Future.wait(
        parsedTracks.map((track) async {
          final centroid = await _computeCentroidFromGpx(track.gpxPath);
          if (centroid == null) return track;
          return track.copyWith(centroid: centroid);
        }),
      );

      tracksById = {for (var t in baseTracks) t.id: t};
    } catch (e) {
      baseTracks = [];
      tracksById = {};
    }
  }

  static Future<List<double>?> _computeCentroidFromGpx(String gpxPath) async {
    try {
      final xmlText = await rootBundle.loadString(gpxPath);
      final doc = xml.XmlDocument.parse(xmlText);
      final trkpts = doc.findAllElements('trkpt');
      if (trkpts.isEmpty) return null;

      double latSum = 0;
      double lonSum = 0;
      int count = 0;

      for (final pt in trkpts) {
        final lat = double.tryParse(pt.getAttribute('lat') ?? '');
        final lon = double.tryParse(pt.getAttribute('lon') ?? '');
        if (lat == null || lon == null) continue;
        latSum += lat;
        lonSum += lon;
        count++;
      }

      if (count == 0) return null;
      return [latSum / count, lonSum / count];
    } catch (e) {
      debugPrint('TracksStore: failed centroid compute for $gpxPath: $e');
      return null;
    }
  }
}

List<Track> get baseTracks => _TracksStore.baseTracks;
Map<String, Track> get tracksById => _TracksStore.tracksById;

Future<void> initTracks() => _TracksStore.init();
