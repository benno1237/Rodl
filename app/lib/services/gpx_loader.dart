import 'package:flutter/services.dart';
import 'package:xml/xml.dart' as xml;

import '../models/gps_point.dart';

/// Load GPX from an asset (local file packaged with the app) and convert to
/// a list of [GPSPoint]. Timestamps are converted to milliseconds since epoch.
class GpxLoader {
  /// Parse GPX content and return track points found in order.
  static List<GPSPoint> parseGpxString(String gpx) {
    final doc = xml.XmlDocument.parse(gpx);
    final trkpts = <xml.XmlElement>[];

    // Typical GPX layout: <gpx><trk><trkseg><trkpt lat.. lon..>...</trkpt></trkseg></trk></gpx>
    for (final trk in doc.findAllElements('trk')) {
      for (final seg in trk.findAllElements('trkseg')) {
        trkpts.addAll(seg.findElements('trkpt'));
      }
    }

    // also support <wpt> if present
    if (trkpts.isEmpty) {
      trkpts.addAll(doc.findAllElements('wpt'));
    }

    final points = <GPSPoint>[];

    for (final e in trkpts) {
      final latAttr = e.getAttribute('lat');
      final lonAttr = e.getAttribute('lon');
      if (latAttr == null || lonAttr == null) continue;
      final lat = double.tryParse(latAttr) ?? 0.0;
      final lon = double.tryParse(lonAttr) ?? 0.0;

      double alt = 0.0;
      final ele = e.getElement('ele')?.innerText;
      if (ele != null) alt = double.tryParse(ele) ?? 0.0;

      int timestampMillis = DateTime.now().millisecondsSinceEpoch;
      final timeText = e.getElement('time')?.innerText;
      if (timeText != null) {
        try {
          timestampMillis = DateTime.parse(timeText).millisecondsSinceEpoch;
        } catch (_) {
          // ignore parse errors and leave now
        }
      }

      // speed is not standard in GPX trkpt; leave as 0.0
      points.add(GPSPoint(
        timestamp: timestampMillis,
        lat: lat,
        lon: lon,
        speedKmh: 0.0,
        altM: alt,
        sats: 0,
        hdop: 99.9,
        age: 0,
      ));
    }

    return points;
  }

  /// Load GPX file from an asset path and parse into points.
  static Future<List<GPSPoint>> loadFromAsset(String assetPath) async {
    final content = await rootBundle.loadString(assetPath);
    return parseGpxString(content);
  }

  /// Slice points by indices [startIndex]..[endIndex] (inclusive start, inclusive end).
  static List<GPSPoint> sliceByIndex(List<GPSPoint> points, int? startIndex, int? endIndex) {
    if (points.isEmpty) return [];
    final s = (startIndex ?? 0).clamp(0, points.length - 1);
    final e = (endIndex ?? (points.length - 1)).clamp(0, points.length - 1);
    if (s > e) return [];
    return points.sublist(s, e + 1);
  }

  /// Slice points by timestamp range (milliseconds since epoch). Nulls are open-ended.
  static List<GPSPoint> sliceByTime(List<GPSPoint> points, int? startMillis, int? endMillis) {
    return points.where((p) {
      if (startMillis != null && p.timestamp < startMillis) return false;
      if (endMillis != null && p.timestamp > endMillis) return false;
      return true;
    }).toList();
  }
}
