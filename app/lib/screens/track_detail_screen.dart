import 'dart:math' as math;

import 'package:flutter/material.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:latlong2/latlong.dart';

import '../models/gps_point.dart';
import '../models/track.dart';
import '../services/gpx_loader.dart';
import '../services/tile_cache.dart';
import '../widgets/leaderboard.dart';
import 'package:provider/provider.dart';
import '../providers/settings_provider.dart';

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
  LatLng? _initialCenter;
  double? _initialZoom;
  int? _selectedSectionIndex;
  int _sectionLoadToken = 0;
  // Matplotlib `tab10` palette (10 distinct colors)
  static const List<Color> _sectionColors = <Color>[
    Color(0xFF1F77B4), // blue
    Color(0xFFFF7F0E), // orange
    Color(0xFF2CA02C), // green
    Color(0xFFD62728), // red
    Color(0xFF9467BD), // purple
    Color(0xFF8C564B), // brown
    Color(0xFFE377C2), // pink
    Color(0xFF7F7F7F), // gray
    Color(0xFFBCBD22), // olive
    Color(0xFF17BECF), // cyan
  ];

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
    final myToken = ++_sectionLoadToken;
    setState(() {
      _loading = true;
      // keep existing points while loading
    });

    try {
      final all = await GpxLoader.loadFromAsset(widget.track.gpxPath);
      if (myToken != _sectionLoadToken) return; // stale

      final s = widget.track.sections.isNotEmpty && index < widget.track.sections.length
          ? widget.track.sections[index]
          : null;

      // keep the full track points so we can draw all sections; update selection together
      setState(() {
        _points = all;
        _loading = false;
        _selectedSectionIndex = index;
      });

      // compute the slice for the selected section and autofit to that slice
      if (s != null) {
        List<GPSPoint> pts = all;
        if (s.startIndex != null || s.endIndex != null) {
          pts = GpxLoader.sliceByIndex(all, s.startIndex ?? 0, s.endIndex ?? all.length - 1);
        } else if (s.startTimeIso != null || s.endTimeIso != null) {
          final int? startMillis = s.startTimeIso != null ? DateTime.tryParse(s.startTimeIso!)?.millisecondsSinceEpoch : null;
          final int? endMillis = s.endTimeIso != null ? DateTime.tryParse(s.endTimeIso!)?.millisecondsSinceEpoch : null;
          pts = GpxLoader.sliceByTime(all, startMillis, endMillis);
        }
        if (myToken == _sectionLoadToken) {
          _fitToPoints(_mapController, pts);
        }
      }
    } catch (e) {
      if (myToken != _sectionLoadToken) return;
      setState(() {
        _loading = false;
        _points = [];
        _selectedSectionIndex = null;
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
    if (lonSpan < 0.000001) lonSpan = 0.000001;

    double latToMerc(double lat) {
      final rad = lat * math.pi / 180;
      return math.log(math.tan(math.pi / 4 + rad / 2));
    }

    final mercMin = latToMerc(minLat);
    final mercMax = latToMerc(maxLat);
    double latFraction = (mercMax - mercMin).abs() / math.pi;
    if (latFraction < 0.000001) latFraction = 0.000001;

    const tileSize = 256.0;
    final ln2 = math.log(2);
    final zLon = math.log(mapWidth * 360 / (lonSpan * tileSize)) / ln2;
    final mapHeight = MediaQuery.of(context).size.height * 0.5;
    final zLat = math.log(mapHeight * math.pi / (latFraction * tileSize)) / ln2;
    double zoom = math.min(zLon, zLat) - 0.5;
    if (zoom.isNaN || zoom.isInfinite) zoom = 13.0;
    zoom = zoom.clamp(1.0, 18.0);

    // store initial center/zoom so the map can start at the fitted view
    _initialCenter = center;
    _initialZoom = zoom;

    WidgetsBinding.instance.addPostFrameCallback((_) {
      try {
        // ignore: avoid_print
        print('TrackDetail: moving to center=$center zoom=$zoom');
        controller.move(center, zoom);
        // ignore: avoid_print
        print('TrackDetail: move requested');
      } catch (e) {
        // ignore: avoid_print
        print('TrackDetail: move failed: $e');
      }
    });
  }

  List<Polyline> _buildSectionPolylines(List<GPSPoint> allPoints, BuildContext ctx) {
    final List<Polyline> out = [];
    final sections = widget.track.sections;
    if (sections.isEmpty) return out;

    // Always draw all sections in their colors. If one is selected,
    // emphasize it by drawing it thicker; otherwise draw all with normal width.
    for (var i = 0; i < sections.length; i++) {
      final s = sections[i];
      final pts = _sliceSectionPoints(allPoints, s);
      if (pts.isNotEmpty) {
        final color = _sectionColors[i % _sectionColors.length];
        if (_selectedSectionIndex != null) {
          final width = (i == _selectedSectionIndex) ? 5 : 2;
          out.add(Polyline(points: pts.map((p) => LatLng(p.lat, p.lon)).toList(), color: color, strokeWidth: width.toDouble()));
        } else {
          out.add(Polyline(points: pts.map((p) => LatLng(p.lat, p.lon)).toList(), color: color, strokeWidth: 4));
        }
      }
    }
    return out;
  }

  List<GPSPoint> _sliceSectionPoints(List<GPSPoint> all, dynamic s) {
    try {
      if ((s.startIndex != null) || (s.endIndex != null)) {
        return GpxLoader.sliceByIndex(all, s.startIndex ?? 0, s.endIndex ?? all.length - 1);
      }
      if ((s.startTimeIso != null) || (s.endTimeIso != null)) {
        final int? startMillis = s.startTimeIso != null ? DateTime.tryParse(s.startTimeIso!)?.millisecondsSinceEpoch : null;
        final int? endMillis = s.endTimeIso != null ? DateTime.tryParse(s.endTimeIso!)?.millisecondsSinceEpoch : null;
        return GpxLoader.sliceByTime(all, startMillis, endMillis);
      }
    } catch (_) {}
    return <GPSPoint>[];
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: Text(widget.track.name)),
      body: Column(
        children: [
          Expanded(
            child: SingleChildScrollView(
              child: Padding(
                padding: const EdgeInsets.all(12),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    const SizedBox(height: 12),
                    Text('Sections', style: Theme.of(context).textTheme.titleMedium?.copyWith(fontWeight: FontWeight.bold)),
                    const SizedBox(height: 8),
                    if (widget.track.sections.isEmpty)
                      const Text('No sections defined')
                    else ...[
                      // 'All' tile: shows whole track partitioned into segments
                      ListTile(
                        contentPadding: const EdgeInsets.symmetric(horizontal: 8.0),
                        title: const Text('All'),
                        leading: SizedBox(
                          width: 48,
                          child: Row(
                            mainAxisAlignment: MainAxisAlignment.start,
                            children: List.generate(
                              widget.track.sections.length.clamp(0, 8),
                              (j) => Container(
                                width: 6,
                                height: 24,
                                margin: const EdgeInsets.only(right: 4),
                                decoration: BoxDecoration(
                                  color: _sectionColors[j % _sectionColors.length],
                                  borderRadius: BorderRadius.circular(2),
                                ),
                              ),
                            ),
                          ),
                        ),
                        tileColor: _selectedSectionIndex == null ? Theme.of(context).colorScheme.primary.withAlpha(20) : null,
                        onTap: () {
                          setState(() {
                            _selectedSectionIndex = null;
                            _loading = true;
                          });
                          if (_points.isEmpty) {
                            _loadAllPoints();
                          } else {
                            setState(() {
                              _loading = false;
                            });
                            _fitToPoints(_mapController, _points);
                          }
                        },
                      ),
                      ...List.generate(widget.track.sections.length, (i) {
                        final s = widget.track.sections[i];
                        final legendColor = _sectionColors[i % _sectionColors.length];
                        return ListTile(
                          contentPadding: const EdgeInsets.symmetric(horizontal: 8.0),
                          minLeadingWidth: 10,
                          leading: Container(
                            width: 6,
                            height: 36,
                            decoration: BoxDecoration(
                              color: legendColor,
                              borderRadius: BorderRadius.circular(2),
                            ),
                          ),
                          title: Text(s.name ?? 'Section ${i + 1}'),
                          tileColor: i == _selectedSectionIndex ? Theme.of(context).colorScheme.primary.withAlpha(20) : null,
                          onTap: () => _loadSectionPoints(i),
                        );
                      }),
                    ],
                    const SizedBox(height: 12),

                    SizedBox(
                      height: 320,
                      child: _loading
                          ? const Center(child: CircularProgressIndicator())
                          : (_points.isEmpty
                              ? const Center(child: Text('No points to display'))
                              : Material(
                                  color: Theme.of(context).colorScheme.surface,
                                  elevation: 2,
                                  shape: RoundedRectangleBorder(
                                    borderRadius: BorderRadius.circular(12.0),
                                    side: BorderSide(color: Colors.black.withValues(alpha: 0.8), width: 2),
                                  ),
                                  clipBehavior: Clip.antiAlias,
                                  child: FlutterMap(
                                    mapController: _mapController,
                                    options: MapOptions(
                                      initialCenter: _initialCenter ?? LatLng(_points[_points.length ~/ 2].lat, _points[_points.length ~/ 2].lon),
                                      initialZoom: _initialZoom ?? 13,
                                      interactionOptions: const InteractionOptions(
                                        flags: InteractiveFlag.all,
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
                                          // full track as subtle background
                                          Polyline(
                                            points: _points.map((p) => LatLng(p.lat, p.lon)).toList(),
                                            color: Theme.of(context).colorScheme.primary.withAlpha(160),
                                            strokeWidth: 3,
                                          ),
                                          // section overlays
                                          ..._buildSectionPolylines(_points, context),
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
                                  ),
                                )),
                    ),
                    const SizedBox(height: 12),
                    // Leaderboard as part of the scrollable content (below the map)
                    Consumer<SettingsProvider>(builder: (ctx, settingsProv, child) {
                      if (!settingsProv.mockLeaderboardEnabled) return const SizedBox.shrink();
                      return Leaderboard(
                        sections: ['Full', ...widget.track.sections.map((s) => s.name ?? 'Section')],
                        selectedIndex: _selectedSectionIndex == null ? 0 : (_selectedSectionIndex! + 1),
                      );
                    }),
                  ],
                ),
              ),
            ),
          ),
 
        ],
      ),
    );
  }
}
