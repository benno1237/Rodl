import 'dart:io';
import 'dart:convert';
import 'dart:typed_data';

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

  /// Returns the total size in bytes of the tile cache directory.
  static Future<int> cacheSizeBytes() async {
    try {
      final root = _cacheRoot ?? (await getApplicationSupportDirectory()).path;
      final dir = Directory(p.join(root, 'map_tiles'));
      if (!await dir.exists()) return 0;
      int total = 0;
      await for (final f in dir.list(recursive: true, followLinks: false)) {
        if (f is File) {
          try {
            total += await f.length();
          } catch (_) {}
        }
      }
      return total;
    } catch (_) {
      return 0;
    }
  }

  /// Deletes the entire tile cache directory.
  static Future<void> clearCache() async {
    try {
      final root = _cacheRoot ?? (await getApplicationSupportDirectory()).path;
      final dir = Directory(p.join(root, 'map_tiles'));
      if (await dir.exists()) {
        await dir.delete(recursive: true);
      }
    } catch (_) {}
  }
}

/// A tile provider that loads tiles packaged as Flutter assets.
class BundledAssetTileProvider extends TileProvider {
  @override
  ImageProvider getImage(dynamic coordinates, dynamic options) {
    final z = coordinates.z;
    final x = coordinates.x;
    final y = coordinates.y;
    // Expect options.urlTemplate to be an asset-style template like
    // 'assets/tiles/<region>/{z}/{x}/{y}.png'
    final urlTemplate = (options as dynamic).urlTemplate as String;
    final path = urlTemplate
        .replaceAll('{z}', z.toString())
        .replaceAll('{x}', x.toString())
        .replaceAll('{y}', y.toString());
    return AssetImage(path);
  }
}

/// Tile provider that only serves tiles from the app support `map_tiles`
/// directory. If a tile file is missing, returns a transparent placeholder
/// image instead of attempting a network fetch.
class FileOnlyTileProvider extends TileProvider {
  static const _transparentPngBase64 =
      'iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR4nGNgYAAAAAMAASsJTYQAAAAASUVORK5CYII=';
  static final _transparentBytes = Uint8List.fromList(base64Decode(_transparentPngBase64));

  @override
  ImageProvider getImage(dynamic coordinates, dynamic options) {
    final z = coordinates.z;
    final x = coordinates.x;
    final y = coordinates.y;
    try {
      final root = CachedNetworkTileProvider._cacheRoot;
      if (root != null) {
        final file = File(p.join(root, 'map_tiles', '$z', '$x', '$y.png'));
        if (file.existsSync()) {
          try {
            // Quick header check to avoid passing invalid data to Flutter's image decoder.
            final raf = file.openSync(mode: FileMode.read);
            try {
              final header = raf.readSync(8);
              // PNG header: 89 50 4E 47 0D 0A 1A 0A
              final isPng = header.length >= 8 && header[0] == 0x89 && header[1] == 0x50 && header[2] == 0x4E && header[3] == 0x47;
              // JPEG header: FF D8 FF
              final isJpeg = header.length >= 3 && header[0] == 0xFF && header[1] == 0xD8 && header[2] == 0xFF;
              if (isPng || isJpeg) {
                return FileImage(file);
              } else {
                // invalid image header, fall back to transparent
                // ignore: avoid_print
                print('Tile cache: invalid image header for ${file.path}, returning transparent');
                return MemoryImage(_transparentBytes);
              }
            } finally {
              raf.closeSync();
            }
          } catch (_) {
            return FileImage(file);
          }
        }
      }
    } catch (_) {}
    return MemoryImage(_transparentBytes);
  }
}
