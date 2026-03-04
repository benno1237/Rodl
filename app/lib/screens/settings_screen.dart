import 'package:flutter/material.dart';
import 'dart:convert';
import 'package:flutter/services.dart';
import 'package:archive/archive.dart';
import 'dart:math' as math;
import 'dart:async';
import 'package:flutter_map/flutter_map.dart';
import 'package:latlong2/latlong.dart';
import '../services/tile_cache.dart';

import 'package:provider/provider.dart';
import '../providers/settings_provider.dart';

class SettingsScreen extends StatefulWidget {
  const SettingsScreen({super.key});

  @override
  State<SettingsScreen> createState() => _SettingsScreenState();
}

class _SettingsScreenState extends State<SettingsScreen> {
  late TextEditingController _usernameController;

  @override
  void initState() {
    super.initState();
    _usernameController = TextEditingController();
    _usernameController.addListener(() {
      // Update provider as the user types.
      try {
        final prov = context.read<SettingsProvider>();
        prov.setUsername(_usernameController.text);
      } catch (_) {}
    });

    WidgetsBinding.instance.addPostFrameCallback((_) {
      context.read<SettingsProvider>().connect();
    });
  }

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    final prov = Provider.of<SettingsProvider>(context);
    final uname = prov.username;
    if (uname != _usernameController.text) {
      _usernameController.text = uname;
    }
  }

  @override
  void dispose() {
    _usernameController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('Settings'), centerTitle: true),
      body: Consumer<SettingsProvider>(
        builder: (context, provider, child) {
          final isConnected = provider.isConnected;

          return ListView(
            padding: const EdgeInsets.all(16),
            children: [
              _buildConnectionStatus(isConnected),
              const SizedBox(height: 24),
              _buildSectionHeader('User Identifier'),
              const SizedBox(height: 12),
              Card(
                elevation: 2,
                shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
                child: Padding(
                  padding: const EdgeInsets.all(16),
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      const Text('Username (unique identifier)', style: TextStyle(fontWeight: FontWeight.w500)),
                      const SizedBox(height: 8),
                      TextField(
                        controller: _usernameController,
                        decoration: const InputDecoration(
                          border: OutlineInputBorder(),
                          labelText: 'Enter username',
                        ),
                      ),
                      const SizedBox(height: 8),
                      Text('This will uniquely identify you in the app.', style: TextStyle(fontSize: 12, color: Colors.grey)),
                    ],
                  ),
                ),
              ),
              const SizedBox(height: 24),
              _buildSectionHeader('Map Tiles Preview'),
              const SizedBox(height: 12),
              _TilePreviewCard(),
              const SizedBox(height: 24),
              _buildSectionHeader('Tile Cache'),
              const SizedBox(height: 12),
              _TileCacheCard(),
            ],
          );
        },
      ),
      endDrawerEnableOpenDragGesture: true,
    );
  }

  Widget _buildConnectionStatus(bool isConnected) {
    return Card(
      elevation: 2,
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Row(
          children: [
            Container(
              width: 12,
              height: 12,
              decoration: BoxDecoration(
                shape: BoxShape.circle,
                color: isConnected ? Colors.green : Colors.orange,
              ),
            ),
            const SizedBox(width: 12),
            Expanded(
              child: Text(
                isConnected ? 'Connected to Rodl' : 'Connecting...',
                style: const TextStyle(fontWeight: FontWeight.w500),
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildSectionHeader(String title) {
    return Text(
      title,
      style: Theme.of(
        context,
      ).textTheme.titleMedium?.copyWith(fontWeight: FontWeight.bold),
    );
  }
}

class _TilePreviewCard extends StatefulWidget {
  @override
  State<_TilePreviewCard> createState() => _TilePreviewCardState();
}

class _TilePreviewCardState extends State<_TilePreviewCard> {
  List<String> _regions = [];
  String? _selectedRegion;
  int _manifestCount = -1;
  

  @override
  void initState() {
    super.initState();
    _loadAssetManifest();
  }

  LatLng? _previewCenter;
  final MapController _previewMapController = MapController();
  List<LatLng> _trackPoints = [];
  Key _mapKey = const ValueKey('preview_map');

  @override
  void dispose() {
    super.dispose();
  }

  Future<void> _loadAssetManifest() async {
    // Gather GPX track asset names from the app manifest (assets/tracks/*.gpx)
    final assetKeys = await _loadAssetKeys();
    // asset keys count (kept silent in release)
    final regions = <String>{};
    for (final k in assetKeys) {
      if (k.startsWith('assets/tracks/') && k.toLowerCase().endsWith('.gpx')) {
        final parts = k.split('/');
        final name = parts.last; // e.g. region.gpx
        final region = name.split('.').first;
        regions.add(region);
      }
    }

    // If the asset manifest didn't list anything (some build modes omit a manifest),
    // probe a small set of known track filenames so the UI still works when assets
    // are present but not enumerated in AssetManifest. Probing is silent.
    if (regions.isEmpty) {
      final knownFiles = [
        'berger_alm.gpx',
        'elfer.gpx',
        'glungezer.gpx',
        'rangger_koepfl.gpx',
      ];
      for (final name in knownFiles) {
        final path = 'assets/tracks/$name';
        try {
          await rootBundle.loadString(path);
          regions.add(name.split('.').first);
        } catch (_) {
          // missing; continue silently
        }
      }
    }

    setState(() {
      _regions = regions.toList()..sort();
      _manifestCount = assetKeys.length;
      if (_regions.isNotEmpty) {
        _selectedRegion = _regions.first;
      }
    });

    // Load GPX overlay for initially selected region (if any)
    if (_selectedRegion != null) {
      try {
        await _loadGpxForRegion(_selectedRegion!);
      } catch (_) {}
      WidgetsBinding.instance.addPostFrameCallback((_) {
        if (!mounted) return;
        try {
          if (_trackPoints.isEmpty && _previewCenter != null) {
            _previewMapController.move(_previewCenter as LatLng, 14.0);
          }
        } catch (_) {}
      });
    }
  }

  Future<void> _loadGpxForRegion(String region) async {
    try {
      String text;
      // Prefer tracks bundled under `assets/tracks/<region>.gpx` or
      // `assets/tracks/<region>/$region.gpx`. Fall back to legacy
      // `assets/tiles/<region>/$region.gpx` if present.
      final candidatePaths = [
        'assets/tracks/$region.gpx',
        'assets/tracks/$region/$region.gpx',
        'assets/tiles/$region/$region.gpx',
      ];
      String? loaded;
      for (final pth in candidatePaths) {
        try {
          loaded = await rootBundle.loadString(pth);
          break;
        } catch (_) {
          // try next
        }
      }
      if (loaded == null) throw Exception('gpx not found');
      text = loaded;

      final tags = RegExp(r'<trkpt\b[^>]*>').allMatches(text);
      final points = <LatLng>[];
      for (final m in tags) {
        final tag = m.group(0)!;
        final latM = RegExp(r'lat="([^"]+)"').firstMatch(tag);
        final lonM = RegExp(r'lon="([^"]+)"').firstMatch(tag);
        if (latM != null && lonM != null) {
          final lat = double.tryParse(latM.group(1)!);
          final lon = double.tryParse(lonM.group(1)!);
          if (lat != null && lon != null) points.add(LatLng(lat, lon));
        }
      }
      if (!mounted) return;
      // compute center of track and update preview center so the map can center on it
      LatLng? center;
      if (points.isNotEmpty) {
        double minLat = points.first.latitude;
        double maxLat = points.first.latitude;
        double minLon = points.first.longitude;
        double maxLon = points.first.longitude;
        for (final pnt in points) {
          minLat = math.min(minLat, pnt.latitude);
          maxLat = math.max(maxLat, pnt.latitude);
          minLon = math.min(minLon, pnt.longitude);
          maxLon = math.max(maxLon, pnt.longitude);
        }
        center = LatLng((minLat + maxLat) / 2.0, (minLon + maxLon) / 2.0);
      }
      setState(() {
        _trackPoints = points;
        if (center != null) _previewCenter = center;
        _mapKey = UniqueKey();
      });
      // adjust viewport to include the whole track
      _fitBoundsToTrack();
    } catch (_) {
      if (!mounted) return;
      setState(() {
        _trackPoints = [];
        _mapKey = UniqueKey();
      });
    }
  }
  @override
  Widget build(BuildContext context) {
    return Card(
      elevation: 2,
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
      child: Padding(
        padding: const EdgeInsets.all(10),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                const Expanded(child: Text('Track Preview', style: TextStyle(fontWeight: FontWeight.w600, fontSize: 16))),
                if (_regions.isNotEmpty)
                  DropdownButton<String>(
                    value: _selectedRegion,
                    items: _regions.map((r) => DropdownMenuItem(value: r, child: Text(r))).toList(),
                    onChanged: (v) async {
                      if (v == null) return;
                      await _loadRegionTiles(v);
                    },
                  ),
              ],
            ),
            const SizedBox(height: 10),
            if (_regions.isEmpty) ...[
              const Text('No tracks found in assets/tracks.'),
              const SizedBox(height: 8),
              Text('Manifest entries: ${_manifestCount < 0 ? "(unknown)" : _manifestCount}'),
              const SizedBox(height: 8),
            ] else ...[
              const SizedBox(height: 8),
              if (_regions.isNotEmpty) ...[
                // Interactive preview map (pan/zoom like rides screen)
                if (_previewCenter != null)
                  SizedBox(
                    height: 300,
                    child: FlutterMap(
                      key: _mapKey,
                      mapController: _previewMapController,
                      options: MapOptions(
                        initialCenter: _previewCenter as LatLng,
                        initialZoom: 14.0,
                        interactionOptions: const InteractionOptions(
                          flags: InteractiveFlag.all & ~InteractiveFlag.rotate,
                        ),
                      ),
                      children: [
                        // Use network tiles for preview (keep GPX overlay from assets)
                        TileLayer(
                          urlTemplate: 'https://tile.openstreetmap.org/{z}/{x}/{y}.png',
                          userAgentPackageName: 'com.rodl.app',
                          tileProvider: CachedNetworkTileProvider(),
                        ),
                        if (_trackPoints.isNotEmpty)
                          PolylineLayer(
                            polylines: [
                              Polyline(
                                points: _trackPoints,
                                color: Theme.of(context).colorScheme.primary,
                                strokeWidth: 4,
                              ),
                            ],
                          ),
                        if (_trackPoints.isNotEmpty)
                          MarkerLayer(
                            markers: [
                              Marker(
                                point: _trackPoints.first,
                                width: 12,
                                height: 12,
                                child: Container(
                                  decoration: BoxDecoration(
                                    color: Colors.red,
                                    shape: BoxShape.circle,
                                    border: Border.all(color: Colors.white, width: 2),
                                  ),
                                ),
                              ),
                              Marker(
                                point: _trackPoints.last,
                                width: 12,
                                height: 12,
                                child: Container(
                                  decoration: BoxDecoration(
                                    color: Colors.green,
                                    shape: BoxShape.circle,
                                    border: Border.all(color: Colors.white, width: 2),
                                  ),
                                ),
                              ),
                            ],
                          ),
                      ],
                    ),
                  )
                else
                  _buildTilesGrid(),
                const SizedBox(height: 8),
              ] else ...[
                const Text('Loading tiles...'),
              ],
            ],
          ],
        ),
      ),
    );
  }

  Future<void> _loadRegionTiles(String region) async {
    // For tracks-based preview, simply load the GPX for the selected region.
    setState(() {
      _selectedRegion = region;
      _trackPoints = [];
      _mapKey = UniqueKey();
    });
    try {
      await _loadGpxForRegion(region);
    } catch (_) {}
    WidgetsBinding.instance.addPostFrameCallback((_) {
      if (!mounted) return;
      try {
        if (_trackPoints.isEmpty && _previewCenter != null) {
          _previewMapController.move(_previewCenter as LatLng, 14.0);
        }
      } catch (_) {}
    });
  }

  Future<List<String>> _loadAssetKeys() async {
    // Try JSON manifest first (debug builds), otherwise fall back to binary.
    try {
      final manifestJson = await rootBundle.loadString('AssetManifest.json');
      final Map<String, dynamic> manifest = json.decode(manifestJson);
      return manifest.keys.toList();
    } catch (_) {
      // Load binary manifest and extract ASCII paths.
      try {
        final data = await rootBundle.load('AssetManifest.bin');
        final bytes = data.buffer.asUint8List();
        final text = String.fromCharCodes(bytes);
        final regex = RegExp(r"assets/(tiles|tracks)/[a-zA-Z0-9_\-/.]+\\.(?:png|jpg|jpeg|gpx)");
        final matches = regex.allMatches(text).map((m) => m.group(0)!).toSet().toList();
        if (matches.isNotEmpty) return matches;
      } catch (_) {}

      // If no direct asset entries found, try bundled zip at assets/images/tiles.zip
      try {
        final zipData = await rootBundle.load('assets/images/tiles.zip');
        final bytes = zipData.buffer.asUint8List();
        final archive = ZipDecoder().decodeBytes(bytes);
        final List<String> entries = [];
        for (final file in archive) {
          if (!file.isFile) continue;
          final name = file.name;
          if (name.startsWith('assets/tiles/') || name.contains('/tiles/') || name.startsWith('assets/tracks/') || name.contains('/tracks/')) {
            entries.add(name);
          }
        }
        return entries;
      } catch (_) {}

      return <String>[];
    }
  }

  Widget _buildTilesGrid() {
    // Tiles grid removed for tracks-based preview; keep placeholder.
    return const SizedBox.shrink();
  }

  void _fitBoundsToTrack() {
    if (_trackPoints.isEmpty) return;
    double minLat = _trackPoints.first.latitude;
    double maxLat = _trackPoints.first.latitude;
    double minLon = _trackPoints.first.longitude;
    double maxLon = _trackPoints.first.longitude;
    for (final p in _trackPoints) {
      minLat = math.min(minLat, p.latitude);
      maxLat = math.max(maxLat, p.latitude);
      minLon = math.min(minLon, p.longitude);
      maxLon = math.max(maxLon, p.longitude);
    }
    final center = LatLng((minLat + maxLat) / 2.0, (minLon + maxLon) / 2.0);
    WidgetsBinding.instance.addPostFrameCallback((_) {
      if (!mounted) return;
      try {
        // determine map pixel size (preview uses fixed height; width from layout)
        final renderBox = context.findRenderObject() as RenderBox?;
        final mapWidth = renderBox?.size.width ?? MediaQuery.of(context).size.width;
        const mapHeight = 300.0;

        // longitude span
        double lonSpan = (maxLon - minLon).abs();
        if (lonSpan < 0.000001) lonSpan = 0.000001;

        // latitude fraction using Web Mercator
        double latToMerc(double lat) {
          final rad = lat * math.pi / 180.0;
          return math.log(math.tan(math.pi / 4.0 + rad / 2.0));
        }

        final mercMin = latToMerc(minLat);
        final mercMax = latToMerc(maxLat);
        double latFraction = (mercMax - mercMin).abs() / math.pi;
        if (latFraction < 0.000001) latFraction = 0.000001;

        const tileSize = 256.0;
        final ln2 = math.log(2);
        final zLon = math.log(mapWidth * 360.0 / (lonSpan * tileSize)) / ln2;
        final zLat = math.log(mapHeight * math.pi / (latFraction * tileSize)) / ln2;
        double zoom = math.min(zLon, zLat) - 0.5; // add a little padding
        if (zoom.isNaN || zoom.isInfinite) zoom = 14.0;
        zoom = zoom.clamp(1.0, 18.0);

        _previewMapController.move(center, zoom);
      } catch (_) {}
    });
  }
}

class _TileCacheCard extends StatefulWidget {
  @override
  State<_TileCacheCard> createState() => _TileCacheCardState();
}

class _TileCacheCardState extends State<_TileCacheCard> {
  int? _sizeBytes;
  bool _isClearing = false;

  @override
  void initState() {
    super.initState();
    _refreshSize();
  }

  Future<void> _refreshSize() async {
    final size = await CachedNetworkTileProvider.cacheSizeBytes();
    if (!mounted) return;
    setState(() {
      _sizeBytes = size;
    });
  }

  Future<void> _clearCache() async {
    setState(() {
      _isClearing = true;
    });
    await CachedNetworkTileProvider.clearCache();
    await _refreshSize();
    if (!mounted) return;
    setState(() {
      _isClearing = false;
    });
    ScaffoldMessenger.of(context).showSnackBar(const SnackBar(content: Text('Tile cache cleared')));
  }

  String _formatBytes(int bytes) {
    if (bytes <= 0) return '0 B';
    const suffixes = ['B', 'KB', 'MB', 'GB'];
    var i = 0;
    double size = bytes.toDouble();
    while (size >= 1024 && i < suffixes.length - 1) {
      size /= 1024;
      i++;
    }
    return '${size.toStringAsFixed(size < 10 ? 1 : 0)} ${suffixes[i]}';
  }

  @override
  Widget build(BuildContext context) {
    final sizeText = _sizeBytes == null ? 'Calculating...' : _formatBytes(_sizeBytes!);
    final canClear = (_sizeBytes ?? 0) > 0 && !_isClearing;

    return Card(
      elevation: 2,
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const Text('Map Tile Cache', style: TextStyle(fontWeight: FontWeight.w500)),
            const SizedBox(height: 12),
            Row(
              children: [
                Expanded(child: Text('Size: $sizeText')),
                if (_isClearing) ...[
                  const SizedBox(width: 12),
                  const SizedBox(width: 20, height: 20, child: CircularProgressIndicator(strokeWidth: 2)),
                ] else ...[
                  ElevatedButton(
                    onPressed: canClear ? _clearCache : null,
                    child: const Text('Clear Cache'),
                  ),
                ],
              ],
            ),
          ],
        ),
      ),
    );
  }
}
