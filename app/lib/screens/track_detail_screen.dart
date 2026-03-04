import 'dart:math' as math;

import 'package:flutter/material.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:latlong2/latlong.dart';

import '../models/gps_point.dart';
import '../models/track.dart';
import '../services/gpx_loader.dart';
import '../services/tile_cache.dart';

class TrackDetailScreen extends StatefulWidget {
  final Track track;

  const TrackDetailScreen({super.key, required this.track});

  @override
  State<TrackDetailScreen> createState() => _TrackDetailScreenState();
}

class _TrackDetailScreenState extends State<TrackDetailScreen> {
  final MapController _mapController = MapController();
  List<GPSPoint> _points = [];
  bool _loading = false;

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addPostFrameCallback((_) {
      _loadAllPoints();
    });
  }

  Future<void> _loadAllPoints() async {
    setState(() {
      _loading = true;
      _points = [];
    });

    try {
      final all = await GpxLoader.loadFromAsset(widget.track.gpxPath);
      setState(() {
        _points = all;
        _loading = false;
      });
      _fitToPoints(_mapController, _points);
    } catch (_) {
      setState(() {
        _loading = false;
        _points = [];
      });
    }
  }

  Future<void> _loadSectionPoints(int index) async {
    setState(() {
      _loading = true;
      _points = [];
    });

    try {
      final all = await GpxLoader.loadFromAsset(widget.track.gpxPath);
      final s = widget.track.sections.isNotEmpty && index < widget.track.sections.length
          ? widget.track.sections[index]
          : null;

      List<GPSPoint> pts = all;
      if (s != null) {
        if (s.startIndex != null || s.endIndex != null) {
          pts = GpxLoader.sliceByIndex(all, s.startIndex ?? 0, s.endIndex ?? all.length - 1);
        } else if (s.startTimeIso != null || s.endTimeIso != null) {
          final int? startMillis = s.startTimeIso != null ? DateTime.tryParse(s.startTimeIso!)?.millisecondsSinceEpoch : null;
          final int? endMillis = s.endTimeIso != null ? DateTime.tryParse(s.endTimeIso!)?.millisecondsSinceEpoch : null;
          pts = GpxLoader.sliceByTime(all, startMillis, endMillis);
        }
      }

      setState(() {
        _points = pts;
        _loading = false;
      });

      _fitToPoints(_mapController, _points);
    } catch (e) {
      setState(() {
        _loading = false;
        _points = [];
      });
    }
  }

  void _fitToPoints(MapController controller, List<GPSPoint> pts) {
    if (pts.isEmpty) return;

    double minLat = pts.first.lat, maxLat = pts.first.lat, minLon = pts.first.lon, maxLon = pts.first.lon;
    for (final p in pts) {
      minLat = math.min(minLat, p.lat);
      maxLat = math.max(maxLat, p.lat);
      minLon = math.min(minLon, p.lon);
      maxLon = math.max(maxLon, p.lon);
    }

    final center = LatLng((minLat + maxLat) / 2, (minLon + maxLon) / 2);

    final mapWidth = MediaQuery.of(context).size.width;
    double lonSpan = (maxLon - minLon).abs();
    if (lonSpan < 000001) lonSpan = 000001;

    double latToMerc(double lat) {
      final rad = lat * math.pi / 180;
      return math.log(math.tan(math.pi / 4 + rad / 2));
    }

    final mercMin = latToMerc(minLat);
    final mercMax = latToMerc(maxLat);
    double latFraction = (mercMax - mercMin).abs() / math.pi;
    if (latFraction < 000001) latFraction = 000001;

    const tileSize = 256;
    final ln2 = math.log(2);
    final zLon = math.log(mapWidth * 360 / (lonSpan * tileSize)) / ln2;
    final mapHeight = MediaQuery.of(context).size.height * 0.5;
    final zLat = math.log(mapHeight * math.pi / (latFraction * tileSize)) / ln2;
    double zoom = math.min(zLon, zLat) - 0.5;
    if (zoom.isNaN || zoom.isInfinite) zoom = 13;
    zoom = zoom.clamp(1, 18);

    WidgetsBinding.instance.addPostFrameCallback((_) {
      try {
        controller.move(center, zoom);
      } catch (_) {}
    });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: Text(widget.track.name)),
      body: SingleChildScrollView(
        child: Padding(
          padding: const EdgeInsets.all(12),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              const SizedBox(height: 12),
              Text('Sections', style: Theme.of(context).textTheme.titleMedium),
              const SizedBox(height: 8),
              if (widget.track.sections.isEmpty)
                const Text('No sections defined')
              else
                ...List.generate(widget.track.sections.length, (i) {
                  final s = widget.track.sections[i];
                  return ListTile(
                    title: Text(s.name ?? 'Section ${i + 1}'),
                    subtitle: Text('id: ${s.id}'),
                    trailing: TextButton(
                      onPressed: () => _loadSectionPoints(i),
                      child: const Text('Load'),
                    ),
                  );
                }),
              const SizedBox(height: 12),

              SizedBox(
                height: 320,
                child: _loading
                    ? const Center(child: CircularProgressIndicator())
                    : (_points.isEmpty
                        ? const Center(child: Text('No points to display'))
                        : FlutterMap(
                            mapController: _mapController,
                            options: MapOptions(
                              initialCenter: LatLng(_points[_points.length ~/ 2].lat, _points[_points.length ~/ 2].lon),
                              initialZoom: 13,
                              interactionOptions: const InteractionOptions(
                                flags: InteractiveFlag.all & ~InteractiveFlag.rotate,
                              ),
                            ),
                            children: [
                              TileLayer(
                                urlTemplate: 'https://tile.openstreetmap.org/{z}/{x}/{y}.png',
                                userAgentPackageName: 'com.rodl.app',
                                tileProvider: CachedNetworkTileProvider(),
                              ),
                              PolylineLayer(
                                polylines: [
                                  Polyline(
                                    points: _points.map((p) => LatLng(p.lat, p.lon)).toList(),
                                    color: Theme.of(context).colorScheme.primary,
                                    strokeWidth: 4,
                                  ),
                                ],
                              ),
                              MarkerLayer(
                                markers: [
                                  Marker(
                                    point: LatLng(_points.first.lat, _points.first.lon),
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
                                    point: LatLng(_points.last.lat, _points.last.lon),
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
                          )),
              ),

              const SizedBox(height: 12),

              Text('Ride Data', style: Theme.of(context).textTheme.titleMedium?.copyWith(fontWeight: FontWeight.bold)),
              const SizedBox(height: 12),

              ListView.builder(
                shrinkWrap: true,
                physics: const NeverScrollableScrollPhysics(),
                itemCount: _points.length,
                itemBuilder: (context, index) {
                  final point = _points[index];
                  // selection state not implemented yet
                  double cardElevation = 1;
                  BorderSide cardSide = BorderSide.none;

                  return Card(
                    elevation: cardElevation,
                    margin: const EdgeInsets.only(bottom: 8),
                    shape: RoundedRectangleBorder(
                      borderRadius: BorderRadius.circular(12),
                      side: cardSide,
                    ),
                    child: Padding(
                      padding: const EdgeInsets.all(12),
                      child: Row(
                        children: [
                          Container(
                            width: 32,
                            height: 32,
                            decoration: BoxDecoration(
                              color: Theme.of(context).colorScheme.primaryContainer,
                              shape: BoxShape.circle,
                            ),
                            child: Center(
                              child: Text(
                                '${index + 1}',
                                style: TextStyle(
                                  fontWeight: FontWeight.bold,
                                  color: Theme.of(context).colorScheme.onPrimaryContainer,
                                  fontSize: 12,
                                ),
                              ),
                            ),
                          ),
                          const SizedBox(width: 12),
                          Expanded(
                            child: Column(
                              crossAxisAlignment: CrossAxisAlignment.start,
                              children: [
                                Text('Point ${index + 1}', style: const TextStyle(fontWeight: FontWeight.w600)),
                                const SizedBox(height: 4),
                                Row(
                                  children: [
                                    _MiniStat(icon: Icons.speed, value: '${point.speedKmh.toStringAsFixed(1)} km/h'),
                                    const SizedBox(width: 12),
                                    _MiniStat(icon: Icons.vibration, value: '${point.acceleration.toStringAsFixed(2)} g'),
                                  ],
                                ),
                              ],
                            ),
                          ),
                          Icon(Icons.chevron_right, color: Theme.of(context).colorScheme.outline),
                        ],
                      ),
                    ),
                  );
                },
              ),
            ],
          ),
        ),
      ),
    );
  }
}

class _MiniStat extends StatelessWidget {
  final IconData icon;
  final String value;

  const _MiniStat({required this.icon, required this.value});

  @override
  Widget build(BuildContext context) {
    return Row(
      children: [
        Icon(icon, size: 14, color: Theme.of(context).colorScheme.onSurfaceVariant),
        const SizedBox(width: 6),
        Text(value, style: Theme.of(context).textTheme.bodySmall),
      ],
    );
  }
}
