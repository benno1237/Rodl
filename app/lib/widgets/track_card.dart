import 'package:flutter/material.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:latlong2/latlong.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'dart:math' as math;

import '../models/track.dart';
import '../services/gpx_loader.dart';
import '../models/gps_point.dart';
import '../services/tile_cache.dart';

class TrackCard extends StatefulWidget {
  final Track track;
  final VoidCallback onDetails;

  const TrackCard({super.key, required this.track, required this.onDetails});

  @override
  State<TrackCard> createState() => _TrackCardState();
}

class _TrackCardState extends State<TrackCard> {
  bool _expanded = false;
  bool _loading = false;
  List<GPSPoint> _points = [];
  int? _bestSeconds;
  final MapController _mapController = MapController();

  Future<void> _toggleExpand() async {
    setState(() => _expanded = !_expanded);
    if (_expanded && _points.isEmpty) {
      await _loadPreview();
    }
  }

  Future<void> _loadPreview() async {
    setState(() {
      _loading = true;
    });
    try {
      final prefs = await SharedPreferences.getInstance();
      _bestSeconds = prefs.getInt('best_time_${widget.track.id}');

      final pts = await GpxLoader.loadFromAsset(widget.track.gpxPath);
      if (!mounted) return;
      setState(() {
        _points = pts;
      });
      if (_points.isNotEmpty) {
        _fitToPoints(_mapController, _points, mapHeight: 200.0);
      }
    } catch (_) {
      // ignore
    } finally {
      if (mounted) setState(() => _loading = false);
    }
  }

  void _fitToPoints(MapController controller, List<GPSPoint> pts, {double mapHeight = 300.0}) {
    if (pts.isEmpty) return;
    double minLat = pts.first.lat, maxLat = pts.first.lat, minLon = pts.first.lon, maxLon = pts.first.lon;
    for (final p in pts) {
      minLat = math.min(minLat, p.lat);
      maxLat = math.max(maxLat, p.lat);
      minLon = math.min(minLon, p.lon);
      maxLon = math.max(maxLon, p.lon);
    }
    final center = LatLng((minLat + maxLat) / 2.0, (minLon + maxLon) / 2.0);

    // compute zoom similar to settings helper
    final mapWidth = MediaQuery.of(context).size.width;
    double lonSpan = (maxLon - minLon).abs();
    if (lonSpan < 0.000001) lonSpan = 0.000001;

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
    double zoom = math.min(zLon, zLat) - 0.5;
    if (zoom.isNaN || zoom.isInfinite) zoom = 13.0;
    zoom = zoom.clamp(1.0, 18.0);

    WidgetsBinding.instance.addPostFrameCallback((_) {
      try {
        controller.move(center, zoom);
      } catch (_) {}
    });
  }

  String _formatSeconds(int s) {
    final mins = s ~/ 60;
    final secs = s % 60;
    return '${mins.toString().padLeft(2, '0')}:${secs.toString().padLeft(2, '0')}';
  }

  @override
  Widget build(BuildContext context) {
    return Card(
      margin: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
      child: Column(
        children: [
          InkWell(
            onTap: _toggleExpand,
            child: Padding(
              padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
              child: Row(
                children: [
                  Expanded(
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: [
                        Text(widget.track.name, style: Theme.of(context).textTheme.titleLarge),
                        if (widget.track.description != null) ...[
                          const SizedBox(height: 6),
                          Text(widget.track.description!, style: Theme.of(context).textTheme.bodySmall),
                        ],
                      ],
                    ),
                  ),
                  IconButton(
                    icon: Icon(_expanded ? Icons.expand_less : Icons.expand_more),
                    onPressed: _toggleExpand,
                  ),
                ],
              ),
            ),
          ),
          if (_expanded) ...[
            SizedBox(
              height: 200,
              child: _loading
                  ? const Center(child: CircularProgressIndicator())
                  : (_points.isEmpty
                      ? const Center(child: Text('No preview available'))
                      : FlutterMap(
                          mapController: _mapController,
                          options: MapOptions(
                            initialCenter: LatLng(_points.first.lat, _points.first.lon),
                            initialZoom: 13.0,
                            interactionOptions: const InteractionOptions(
                              flags: InteractiveFlag.pinchZoom | InteractiveFlag.drag,
                            ),
                          ),
                          children: [
                            TileLayer(
                              urlTemplate: 'https://tile.openstreetmap.org/{z}/{x}/{y}.png',
                              userAgentPackageName: 'com.rodl.app',
                              tileProvider: CachedNetworkTileProvider(),
                            ),
                            if (_points.isNotEmpty)
                              PolylineLayer(
                                polylines: [
                                  Polyline(
                                    points: _points.map((p) => LatLng(p.lat, p.lon)).toList(),
                                    color: Theme.of(context).colorScheme.primary,
                                    strokeWidth: 3,
                                  ),
                                ],
                              ),
                          ],
                        )),
            ),
            // show details button and best time only when expanded
            Padding(
              padding: const EdgeInsets.symmetric(horizontal: 12.0, vertical: 8.0),
              child: Row(
                children: [
                  ElevatedButton(
                    onPressed: widget.onDetails,
                    child: const Text('Details'),
                  ),
                  const Spacer(),
                  Text('Best: ${_bestSeconds != null ? _formatSeconds(_bestSeconds!) : '--:--'}', style: Theme.of(context).textTheme.bodyMedium),
                ],
              ),
            ),
          ],
        ],
      ),
    );
  }
}
