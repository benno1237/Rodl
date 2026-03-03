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
  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addPostFrameCallback((_) {
      context.read<SettingsProvider>().connect();
    });
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
              _buildSectionHeader('Map Tiles Preview'),
              const SizedBox(height: 12),
              _TilePreviewCard(),
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
  Map<int, List<String>> _tilesByZoom = {};
  int? _selectedZoom;
  double _zoom = 14.0;
  Timer? _zoomDebounce;

  @override
  void initState() {
    super.initState();
    _loadAssetManifest();
  }

  LatLng? _previewCenter;
  final MapController _previewMapController = MapController();

  @override
  void dispose() {
    _zoomDebounce?.cancel();
    super.dispose();
  }

  Future<void> _loadAssetManifest() async {
    final tileKeys = await _loadAssetKeys();

    final regions = <String>{};
    final Map<String, Map<int, List<String>>> temp = {};

    for (final k in tileKeys) {
      // path: assets/tiles/<region>/<z>/<x>/<y.png>
      final parts = k.split('/');
      if (parts.length >= 6) {
        final region = parts[2];
        final z = int.tryParse(parts[3]);
        if (z != null) {
          regions.add(region);
          temp.putIfAbsent(region, () => {});
          temp[region]!.putIfAbsent(z, () => []);
          temp[region]![z]!.add(k);
        }
      }
    }

    setState(() {
      _regions = regions.toList()..sort();
      if (_regions.isNotEmpty) {
        _selectedRegion = _regions.first;
        _tilesByZoom = (temp[_selectedRegion!] ?? {}).map((k, v) => MapEntry(k, v));
        if (_tilesByZoom.isNotEmpty) {
          final zs = _tilesByZoom.keys.toList()..sort();
          _selectedZoom = zs.last;
          _zoom = (_selectedZoom ?? 14).toDouble();
          // compute a preview center from first tile
          final sample = (_tilesByZoom[_selectedZoom!] ?? []).first;
          _previewCenter = _tilePathToLatLng(sample);
          WidgetsBinding.instance.addPostFrameCallback((_) {
            if (!mounted) return;
            try {
              _previewMapController.move(_previewCenter as LatLng, _zoom);
            } catch (_) {}
          });
        }
      }
    });
  }

  @override
  Widget build(BuildContext context) {
    return Card(
      elevation: 2,
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const Text('Bundled Map Tiles', style: TextStyle(fontWeight: FontWeight.w500)),
            const SizedBox(height: 12),
            if (_regions.isEmpty) ...[
              const Text('No bundled tiles found.'),
            ] else ...[
              DropdownButton<String>(
                value: _selectedRegion,
                items: _regions.map((r) => DropdownMenuItem(value: r, child: Text(r))).toList(),
                onChanged: (v) {
                  if (v == null) return;
                  setState(() {
                    _selectedRegion = v;
                    _tilesByZoom.clear();
                    _selectedZoom = null;
                  });
                  _loadRegionTiles(v);
                },
              ),
              const SizedBox(height: 8),
              if (_tilesByZoom.isNotEmpty) ...[
                // Interactive preview map (pan/zoom like rides screen)
                if (_previewCenter != null)
                  SizedBox(
                    height: 300,
                    child: FlutterMap(
                      mapController: _previewMapController,
                      options: MapOptions(
                        initialCenter: _previewCenter as LatLng,
                        initialZoom: (_selectedZoom ?? 14).toDouble(),
                        interactionOptions: const InteractionOptions(
                          flags: InteractiveFlag.all & ~InteractiveFlag.rotate,
                        ),
                      ),
                      children: [
                        TileLayer(
                          urlTemplate: 'https://tile.openstreetmap.org/{z}/{x}/{y}.png',
                          tileProvider: CachedNetworkTileProvider(),
                        ),
                        PolylineLayer(
                          polylines: <Polyline<Object>>[],
                        ),
                      ],
                    ),
                  )
                else
                  _buildTilesGrid(),
                const SizedBox(height: 8),
                Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Row(
                      mainAxisAlignment: MainAxisAlignment.spaceBetween,
                      children: [
                        const Text('Preview zoom', style: TextStyle(fontWeight: FontWeight.w500)),
                        Text(_zoom.toStringAsFixed(0), style: TextStyle(color: Theme.of(context).colorScheme.primary, fontWeight: FontWeight.bold)),
                      ],
                    ),
                    Slider(
                      value: _zoom.clamp(0, 19),
                      min: 0,
                      max: 19,
                      divisions: 19,
                      label: _zoom.toStringAsFixed(0),
                      onChanged: (v) {
                        setState(() {
                          _zoom = v;
                        });
                        // move preview map immediately
                        if (_previewCenter != null) {
                          try {
                            _previewMapController.move(_previewCenter as LatLng, _zoom);
                          } catch (_) {}
                        }
                        // debounce extraction
                        _zoomDebounce?.cancel();
                        _zoomDebounce = Timer(const Duration(milliseconds: 500), () {
                          if (!mounted) return;
                          if (_selectedRegion != null) {
                            final provider = context.read<SettingsProvider>();
                            provider.previewExtractTiles(_selectedRegion!, _zoom.round());
                            setState(() {
                              _selectedZoom = _zoom.round();
                            });
                          }
                        });
                      },
                    ),
                  ],
                ),
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
    final tileKeys = (await _loadAssetKeys())
      .where((k) => k.startsWith('assets/tiles/$region/'))
      .toList();

    final Map<int, List<String>> temp = {};
    for (final k in tileKeys) {
      final parts = k.split('/');
      if (parts.length >= 5) {
        final z = int.tryParse(parts[3]);
        if (z != null) {
          temp.putIfAbsent(z, () => []);
          temp[z]!.add(k);
        }
      }
    }

    setState(() {
      _tilesByZoom = temp.map((k, v) => MapEntry(k, v));
      if (_tilesByZoom.isNotEmpty) {
        final zs = _tilesByZoom.keys.toList()..sort();
        _selectedZoom = zs.last;
        final sample = (_tilesByZoom[_selectedZoom!] ?? []).first;
        _previewCenter = _tilePathToLatLng(sample);
        WidgetsBinding.instance.addPostFrameCallback((_) {
          if (!mounted) return;
          try {
            _previewMapController.move(_previewCenter as LatLng, (_selectedZoom ?? 14).toDouble());
          } catch (_) {}
        });
        // trigger preview extraction
        if (_selectedRegion != null && _selectedZoom != null) {
          final provider = context.read<SettingsProvider>();
          provider.previewExtractTiles(_selectedRegion!, _selectedZoom!);
        }
      }
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
        final regex = RegExp(r"assets/tiles/[a-zA-Z0-9_\-/.]+\\.(?:png|jpg|jpeg)");
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
          if (name.startsWith('assets/tiles/') || name.contains('/tiles/')) {
            // normalize to assets/tiles/...
            final normalized = name.replaceAll('assets/tiles', 'assets/tiles');
            entries.add(normalized);
          }
        }
        return entries;
      } catch (_) {}

      return <String>[];
    }
  }

  Widget _buildTilesGrid() {
    if (_selectedZoom == null) return const SizedBox.shrink();
    final tiles = _tilesByZoom[_selectedZoom!] ?? [];
    final sample = tiles.take(9).toList();
    return SizedBox(
      height: 180,
      child: GridView.count(
        crossAxisCount: 3,
        children: sample.map((p) => Padding(
          padding: const EdgeInsets.all(4),
          child: Image.asset(p, fit: BoxFit.cover),
        )).toList(),
      ),
    );
  }

  LatLng? _tilePathToLatLng(String path) {
    // path: assets/tiles/<region>/<z>/<x>/<y.png>
    final parts = path.split('/');
    if (parts.length < 6) return null;
    final z = int.tryParse(parts[3]);
    final x = int.tryParse(parts[4]);
    final yStr = parts[5];
    final y = int.tryParse(yStr.split('.').first);
    if (z == null || x == null || y == null) return null;

    final n = math.pow(2.0, z).toDouble();
    final lon = x / n * 360.0 - 180.0;
    final t = math.pi * (1 - 2 * y / n);
    final s = (math.exp(t) - math.exp(-t)) / 2.0; // sinh(t)
    final latRad = math.atan(s);
    final lat = latRad * 180.0 / math.pi;
    return LatLng(lat, lon);
  }
}
