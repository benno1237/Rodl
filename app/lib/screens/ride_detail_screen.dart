import 'package:flutter/material.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:latlong2/latlong.dart';
import 'package:intl/intl.dart';
import '../models/ride.dart';
import '../models/gps_point.dart';

class RideDetailScreen extends StatefulWidget {
  final Ride ride;

  const RideDetailScreen({super.key, required this.ride});

  @override
  State<RideDetailScreen> createState() => _RideDetailScreenState();
}

class _RideDetailScreenState extends State<RideDetailScreen> {
  final MapController _mapController = MapController();
  int? _selectedPointIndex;

  @override
  Widget build(BuildContext context) {
    final points = widget.ride.points;
    final center = points.isNotEmpty
        ? LatLng(points[points.length ~/ 2].lat, points[points.length ~/ 2].lon)
        : const LatLng(47.3769, 8.5417);

    return Scaffold(
      body: CustomScrollView(
        slivers: [
          SliverAppBar(
            expandedHeight: 300,
            pinned: true,
            flexibleSpace: FlexibleSpaceBar(
              title: Text(
                DateFormat('HH:mm').format(widget.ride.startTime),
                style: const TextStyle(
                  color: Colors.white,
                  fontWeight: FontWeight.bold,
                  shadows: [Shadow(color: Colors.black54, blurRadius: 4)],
                ),
              ),
              background: FlutterMap(
                mapController: _mapController,
                options: MapOptions(
                  initialCenter: center,
                  initialZoom: 14,
                  interactionOptions: const InteractionOptions(
                    flags: InteractiveFlag.all & ~InteractiveFlag.rotate,
                  ),
                ),
                children: [
                  TileLayer(
                    urlTemplate:
                        'https://tile.openstreetmap.org/{z}/{x}/{y}.png',
                    userAgentPackageName: 'com.rodl.app',
                  ),
                  PolylineLayer(
                    polylines: [
                      Polyline(
                        points: points
                            .map((p) => LatLng(p.lat, p.lon))
                            .toList(),
                        color: Theme.of(context).colorScheme.primary,
                        strokeWidth: 4,
                      ),
                    ],
                  ),
                  if (_selectedPointIndex != null)
                    MarkerLayer(
                      markers: [
                        Marker(
                          point: LatLng(
                            points[_selectedPointIndex!].lat,
                            points[_selectedPointIndex!].lon,
                          ),
                          width: 20,
                          height: 20,
                          child: Container(
                            decoration: BoxDecoration(
                              color: Colors.red,
                              shape: BoxShape.circle,
                              border: Border.all(color: Colors.white, width: 2),
                            ),
                          ),
                        ),
                      ],
                    ),
                ],
              ),
            ),
          ),
          SliverToBoxAdapter(
            child: Padding(
              padding: const EdgeInsets.all(16),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  _buildStatsRow(context),
                  const SizedBox(height: 24),
                  Text(
                    'Ride Data',
                    style: Theme.of(context).textTheme.titleMedium?.copyWith(
                      fontWeight: FontWeight.bold,
                    ),
                  ),
                  const SizedBox(height: 12),
                  _buildPointList(points),
                ],
              ),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildStatsRow(BuildContext context) {
    final ride = widget.ride;
    return Card(
      elevation: 2,
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Row(
          mainAxisAlignment: MainAxisAlignment.spaceAround,
          children: [
            _StatTile(
              icon: Icons.timer_outlined,
              label: 'Duration',
              value: ride.formattedDuration,
            ),
            Container(
              width: 1,
              height: 40,
              color: Theme.of(context).colorScheme.outlineVariant,
            ),
            _StatTile(
              icon: Icons.speed,
              label: 'Max Speed',
              value: '${ride.maxSpeed.toStringAsFixed(1)} km/h',
            ),
            Container(
              width: 1,
              height: 40,
              color: Theme.of(context).colorScheme.outlineVariant,
            ),
            _StatTile(
              icon: Icons.straighten,
              label: 'Distance',
              value: '${(ride.totalDistanceKm * 1000).toStringAsFixed(0)} m',
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildPointList(List<GPSPoint> points) {
    return ListView.builder(
      shrinkWrap: true,
      physics: const NeverScrollableScrollPhysics(),
      itemCount: points.length,
      itemBuilder: (context, index) {
        final point = points[index];
        final isSelected = _selectedPointIndex == index;

        return Card(
          elevation: isSelected ? 4 : 1,
          margin: const EdgeInsets.only(bottom: 8),
          shape: RoundedRectangleBorder(
            borderRadius: BorderRadius.circular(12),
            side: isSelected
                ? BorderSide(
                    color: Theme.of(context).colorScheme.primary,
                    width: 2,
                  )
                : BorderSide.none,
          ),
          child: InkWell(
            borderRadius: BorderRadius.circular(12),
            onTap: () {
              setState(() {
                _selectedPointIndex = isSelected ? null : index;
              });
              if (!isSelected) {
                _mapController.move(LatLng(point.lat, point.lon), 16);
              }
            },
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
                          color: Theme.of(
                            context,
                          ).colorScheme.onPrimaryContainer,
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
                        Text(
                          'Point ${index + 1}',
                          style: const TextStyle(fontWeight: FontWeight.w600),
                        ),
                        const SizedBox(height: 4),
                        Row(
                          children: [
                            _MiniStat(
                              icon: Icons.speed,
                              value:
                                  '${point.speedKmh.toStringAsFixed(1)} km/h',
                            ),
                            const SizedBox(width: 12),
                            _MiniStat(
                              icon: Icons.vibration,
                              value:
                                  '${point.acceleration.toStringAsFixed(2)} g',
                            ),
                          ],
                        ),
                      ],
                    ),
                  ),
                  Icon(
                    Icons.chevron_right,
                    color: Theme.of(context).colorScheme.outline,
                  ),
                ],
              ),
            ),
          ),
        );
      },
    );
  }
}

class _StatTile extends StatelessWidget {
  final IconData icon;
  final String label;
  final String value;

  const _StatTile({
    required this.icon,
    required this.label,
    required this.value,
  });

  @override
  Widget build(BuildContext context) {
    return Column(
      children: [
        Icon(icon, color: Theme.of(context).colorScheme.primary),
        const SizedBox(height: 4),
        Text(
          label,
          style: TextStyle(
            fontSize: 11,
            color: Theme.of(context).colorScheme.outline,
          ),
        ),
        const SizedBox(height: 2),
        Text(
          value,
          style: const TextStyle(fontWeight: FontWeight.bold, fontSize: 14),
        ),
      ],
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
      mainAxisSize: MainAxisSize.min,
      children: [
        Icon(icon, size: 14, color: Theme.of(context).colorScheme.outline),
        const SizedBox(width: 4),
        Text(
          value,
          style: TextStyle(
            fontSize: 12,
            color: Theme.of(context).colorScheme.outline,
          ),
        ),
      ],
    );
  }
}
