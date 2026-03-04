import 'dart:convert';

import 'package:flutter/services.dart';

import '../models/track.dart';

class _TracksStore {
  static List<Track> baseTracks = [];
  static Map<String, Track> tracksById = {};

  static Future<void> init() async {
    try {
      final raw = await rootBundle.loadString('assets/data/tracks.json');
      final list = json.decode(raw) as List<dynamic>;
      baseTracks = list.map((e) => Track.fromJson(e as Map<String, dynamic>)).toList();
      tracksById = {for (var t in baseTracks) t.id: t};
    } catch (e) {
      baseTracks = [];
      tracksById = {};
    }
  }
}

List<Track> get baseTracks => _TracksStore.baseTracks;
Map<String, Track> get tracksById => _TracksStore.tracksById;

Future<void> initTracks() => _TracksStore.init();
