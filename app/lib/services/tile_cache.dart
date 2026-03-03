import 'dart:io';

import 'package:flutter/widgets.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:path/path.dart' as p;
import 'package:path_provider/path_provider.dart';
import 'package:http/http.dart' as http;

class CachedNetworkTileProvider extends TileProvider {
  static String? _cacheRoot;
  static Future<void>? _initFuture;

  CachedNetworkTileProvider() {
    _initFuture ??= _init();
  }

  /// Ensure cache directory is initialized. Call and await this in `main()`
  /// before `runApp()` to guarantee cached tiles are available immediately.
  static Future<void> init() => _initFuture ??= _init();

  static Future<void> _init() async {
    try {
      final d = await getApplicationSupportDirectory();
      _cacheRoot = d.path;
    } catch (_) {
      _cacheRoot = null;
    }
  }

  @override
  ImageProvider getImage(dynamic coordinates, dynamic options) {
    final z = coordinates.z;
    final x = coordinates.x;
    final y = coordinates.y;
    final urlTemplate = (options as dynamic).urlTemplate as String;
    final url = urlTemplate
        .replaceAll('{z}', z.toString())
        .replaceAll('{x}', x.toString())
        .replaceAll('{y}', y.toString());

    if (_cacheRoot != null) {
      final file = File(p.join(_cacheRoot!, 'map_tiles', '$z', '$x', '$y.png'));
      if (file.existsSync()) {
        return FileImage(file);
      } else {
        // Start background download and return network image for now
        _downloadAndSave(url, file);
        return NetworkImage(url);
      }
    } else {
      // Cache directory not yet initialized — download later and return network image
      _downloadAndSaveLater(url, z, x, y);
      return NetworkImage(url);
    }
  }

  static Future<void> _downloadAndSave(String url, File file) async {
    try {
      final res = await http.get(Uri.parse(url));
      if (res.statusCode == 200) {
        await file.create(recursive: true);
        await file.writeAsBytes(res.bodyBytes);
      }
    } catch (_) {}
  }

  static Future<void> _downloadAndSaveLater(String url, int z, int x, int y) async {
    try {
      final dir = await getApplicationSupportDirectory();
      final file = File(p.join(dir.path, 'map_tiles', '$z', '$x', '$y.png'));
      await _downloadAndSave(url, file);
    } catch (_) {}
  }
}
