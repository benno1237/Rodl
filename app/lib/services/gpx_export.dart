import 'dart:io';
import 'package:xml/xml.dart' as xml;
import 'package:path_provider/path_provider.dart';
import 'package:path/path.dart' as p;
import '../models/ride.dart';

Future<String> exportRideToGpx(Ride ride) async {
  // Build GPX document
  final builder = xml.XmlBuilder();
  builder.processing('xml', 'version="1.0" encoding="UTF-8"');
  builder.element('gpx', nest: () {
    builder.attribute('version', '1.1');
    builder.attribute('creator', 'Rodl');
    builder.attribute('xmlns', 'http://www.topografix.com/GPX/1/1');

    // metadata with start time
    builder.element('metadata', nest: () {
      builder.element('time', nest: ride.startTime.toUtc().toIso8601String());
    });

    // track
    builder.element('trk', nest: () {
      builder.element('name', nest: ride.name ?? 'Ride');
      builder.element('trkseg', nest: () {
        if (ride.points.isNotEmpty) {
          final firstTs = ride.points.first.timestamp;
          for (final pnt in ride.points) {
            // determine absolute time: if timestamp looks absolute use it,
            // otherwise derive from ride.startTime and relative offset
            DateTime t;
            if (pnt.timestamp > 100000000000) {
              t = DateTime.fromMillisecondsSinceEpoch(pnt.timestamp).toUtc();
            } else {
              final offset = pnt.timestamp - firstTs;
              t = ride.startTime.add(Duration(milliseconds: offset)).toUtc();
            }

            builder.element('trkpt', nest: () {
              builder.attribute('lat', pnt.lat.toString());
              builder.attribute('lon', pnt.lon.toString());
              builder.element('ele', nest: pnt.altM.toString());
              builder.element('time', nest: t.toIso8601String());
              builder.element('speed', nest: (pnt.speedKmh / 3.6).toString());
            });
          }
        }
      });
    });
  });

  final doc = builder.buildDocument();
  final gpxString = doc.toXmlString(pretty: true, indent: '  ');

  // Determine a writable directory on Android/iOS: app-specific external directory
  final dir = await getExternalStorageDirectory();
  final baseDir = dir ?? await getApplicationDocumentsDirectory();

  final safeName = (ride.name != null && ride.name!.isNotEmpty)
      ? ride.name!.replaceAll(RegExp(r'[\\/:*?"<>|]'), '_')
      : 'ride';
  final ts = ride.startTime.toIso8601String().replaceAll(RegExp(r'[:\.]'), '-');
  final filename = '${safeName}_$ts.gpx';

  final outPath = p.join(baseDir.path, filename);
  final outFile = File(outPath);
  await outFile.create(recursive: true);
  await outFile.writeAsString(gpxString, flush: true);

  return outFile.path;
}
