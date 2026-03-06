import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
// flutter_slidable no longer used in this file (in-card actions implemented).
import 'package:intl/intl.dart';
import '../providers/rides_provider.dart';
import '../data/sleds.dart';
import 'ride_detail_screen.dart';
import '../services/gpx_export.dart';

class DayDetailScreen extends StatefulWidget {
  final DateTime date;

  const DayDetailScreen({super.key, required this.date});

  @override
  State<DayDetailScreen> createState() => _DayDetailScreenState();
}

class _DayDetailScreenState extends State<DayDetailScreen> {
  int? _revealedRideId;
  double _dragDx = 0.0;

  void _onDragStart() {
    _dragDx = 0.0;
  }

  void _onDragUpdate(DragUpdateDetails details, int rideId) {
    _dragDx += details.delta.dx;
    // if swiped left sufficiently, reveal actions
    if (_dragDx <= -20) {
      setState(() => _revealedRideId = rideId);
    }
    // if swiped right, hide
    if (_dragDx >= 20 && _revealedRideId == rideId) {
      setState(() => _revealedRideId = null);
    }
  }

  void _onDragEnd(DragEndDetails details, int rideId) {
    final v = details.primaryVelocity;
    if (v != null) {
      // faster threshold for intentional swipes
      if (v < -300) {
        setState(() => _revealedRideId = rideId);
      } else if (v > 300 && _revealedRideId == rideId) {
        setState(() => _revealedRideId = null);
      }
    } else {
      // fallback to distance
      if (_dragDx <= -20) setState(() => _revealedRideId = rideId);
      if (_dragDx >= 20 && _revealedRideId == rideId) setState(() => _revealedRideId = null);
    }
    _dragDx = 0.0;
  }

  

  @override
  Widget build(BuildContext context) {
    final rides = context.watch<RidesProvider>().getRidesForDay(widget.date);

    return Scaffold(
      appBar: AppBar(
        title: Text(DateFormat('MMM d, yyyy').format(widget.date)),
        centerTitle: true,
      ),
      body: ListView.builder(
        padding: const EdgeInsets.all(16),
        itemCount: rides.length + 1,
        itemBuilder: (context, index) {
          if (index == rides.length) {
            return Padding(
              padding: const EdgeInsets.only(top: 8.0, bottom: 24.0, right: 8.0),
              child: Align(
                alignment: Alignment.centerRight,
                child: Column(
                  mainAxisSize: MainAxisSize.min,
                  crossAxisAlignment: CrossAxisAlignment.stretch,
                  children: [
                    Text(
                      'Swipe left on ride for options',
                      textAlign: TextAlign.right,
                      style: Theme.of(context).textTheme.bodySmall?.copyWith(color: Theme.of(context).colorScheme.outline),
                    ),
                    const SizedBox(height: 2),
                    Align(
                      alignment: Alignment.centerRight,
                      child: Text(
                        '←',
                        style: Theme.of(context).textTheme.bodySmall?.copyWith(color: Theme.of(context).colorScheme.outline, fontSize: 12),
                      ),
                    ),
                  ],
                ),
              ),
            );
          }

          final ride = rides[index];
          final startTime = DateFormat('HH:mm').format(ride.startTime);

            return Padding(
            padding: const EdgeInsets.only(bottom: 12),
            child: GestureDetector(
              onHorizontalDragStart: (_) => _onDragStart(),
              onHorizontalDragUpdate: (d) => _onDragUpdate(d, ride.id),
              onHorizontalDragEnd: (d) => _onDragEnd(d, ride.id),
              child: Card(
                margin: EdgeInsets.zero,
                elevation: 2,
                shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
                child: Stack(
                  children: [
                    // Main content
                    InkWell(
                      borderRadius: BorderRadius.circular(16),
                      onTap: () {
                        if (_revealedRideId != null) {
                          setState(() => _revealedRideId = null);
                          return;
                        }
                        Navigator.push(
                          context,
                          MaterialPageRoute(
                            builder: (_) => RideDetailScreen(ride: ride),
                          ),
                        );
                      },
                      child: Padding(
                        padding: const EdgeInsets.all(16),
                        child: Column(
                          crossAxisAlignment: CrossAxisAlignment.start,
                          children: [
                                Row(
                              mainAxisAlignment: MainAxisAlignment.spaceBetween,
                              children: [
                                Row(
                                  children: [
                                    Icon(
                                      Icons.two_wheeler,
                                      color: Theme.of(context).colorScheme.primary,
                                    ),
                                    const SizedBox(width: 8),
                                    Text(
                                      ride.name ?? 'Ride ${index + 1}',
                                      style: Theme.of(context).textTheme.titleMedium?.copyWith(fontWeight: FontWeight.bold),
                                    ),
                                  ],
                                ),
                                Text(
                                  startTime,
                                  style: TextStyle(color: Theme.of(context).colorScheme.outline),
                                ),
                              ],
                            ),
                            const SizedBox(height: 6),
                            // Show sled used for this ride if available
                            if (ride.sledId != null)
                              Builder(builder: (ctx) {
                                final sled = sledsById[ride.sledId];
                                final color = sled?.primaryColor ?? Theme.of(ctx).colorScheme.outline;
                                final label = sled?.name ?? 'Unknown sled';
                                return Row(
                                  children: [
                                    Icon(Icons.two_wheeler, size: 14, color: color),
                                    const SizedBox(width: 8),
                                    Text(
                                      label,
                                      style: Theme.of(ctx).textTheme.bodySmall?.copyWith(color: color),
                                    ),
                                  ],
                                );
                              }),
                            const SizedBox(height: 12),
                            Row(
                              children: [
                                _StatItem(icon: Icons.timer_outlined, label: 'Duration', value: ride.formattedDuration),
                                const SizedBox(width: 24),
                                _StatItem(icon: Icons.speed, label: 'Max Speed', value: '${ride.maxSpeed.toStringAsFixed(1)} km/h'),
                                const SizedBox(width: 24),
                                _StatItem(icon: Icons.straighten, label: 'Distance', value: '${(ride.totalDistanceKm * 1000).toStringAsFixed(0)} m'),
                              ],
                            ),
                          ],
                        ),
                      ),
                    ),

                    // Sliding overlay actions (cover ~50% of card from right)
                    Positioned.fill(
                      child: Align(
                        alignment: Alignment.centerRight,
                        child: FractionallySizedBox(
                          widthFactor: 0.5,
                          alignment: Alignment.centerRight,
                          child: AnimatedSlide(
                            duration: const Duration(milliseconds: 200),
                            curve: Curves.easeOutCubic,
                            offset: _revealedRideId == ride.id ? Offset.zero : const Offset(1.15, 0.0),
                            child: ClipRRect(
                              borderRadius: BorderRadius.circular(16),
                              child: Container(
                                color: Color.fromRGBO(0, 0, 0, 0.05),
                                child: Row(
                                  children: [
                                    // Delete action (left)
                                    Expanded(
                                      flex: 1,
                                      child: InkWell(
                                        onTap: () async {
                                          final ridesProv = context.read<RidesProvider>();
                                          final messenger = ScaffoldMessenger.of(context);
                                          final confirmed = await showDialog<bool>(
                                            context: context,
                                            builder: (dctx) => AlertDialog(
                                              title: const Text('Delete ride'),
                                              content: const Text('Are you sure you want to delete this ride?'),
                                              actions: [
                                                TextButton(onPressed: () => Navigator.of(dctx).pop(false), child: const Text('Cancel')),
                                                TextButton(onPressed: () => Navigator.of(dctx).pop(true), child: const Text('Delete')),
                                              ],
                                            ),
                                          );
                                          if (!mounted) return;
                                          if (confirmed == true) {
                                            ridesProv.deleteRide(ride.id);
                                            messenger.showSnackBar(SnackBar(content: Text('Ride deleted'), duration: Duration(milliseconds: 1500)));
                                          }
                                          setState(() => _revealedRideId = null);
                                        },
                                        child: Container(
                                          color: Colors.red,
                                          alignment: Alignment.center,
                                          child: Column(
                                            mainAxisSize: MainAxisSize.min,
                                            children: const [
                                              Icon(Icons.delete, color: Colors.white),
                                              SizedBox(height: 4),
                                              Text('Delete', style: TextStyle(color: Colors.white)),
                                            ],
                                          ),
                                        ),
                                      ),
                                    ),

                                    // Export action (middle)
                                    Expanded(
                                      flex: 1,
                                      child: InkWell(
                                        onTap: () async {
                                          final messenger = ScaffoldMessenger.of(context);
                                          try {
                                            final path = await exportRideToGpx(ride);
                                            if (!mounted) return;
                                            messenger.showSnackBar(SnackBar(content: Text('Exported GPX to $path'), duration: Duration(seconds: 3)));
                                          } catch (e) {
                                            if (!mounted) return;
                                            messenger.showSnackBar(SnackBar(content: Text('Export failed: $e'), duration: Duration(seconds: 3)));
                                          }
                                          setState(() => _revealedRideId = null);
                                        },
                                        child: Container(
                                          color: Colors.green,
                                          alignment: Alignment.center,
                                          child: Column(
                                            mainAxisSize: MainAxisSize.min,
                                            children: const [
                                              Icon(Icons.share, color: Colors.white),
                                              SizedBox(height: 4),
                                              Text('Export', style: TextStyle(color: Colors.white)),
                                            ],
                                          ),
                                        ),
                                      ),
                                    ),

                                    // Rename action (right)
                                    Expanded(
                                      flex: 1,
                                      child: InkWell(
                                        onTap: () async {
                                          final controller = TextEditingController(text: ride.name ?? '');
                                          final ridesProv = context.read<RidesProvider>();
                                          final messenger = ScaffoldMessenger.of(context);
                                          final result = await showDialog<String?>(
                                            context: context,
                                            builder: (dctx) => AlertDialog(
                                              title: const Text('Rename ride'),
                                              content: TextField(controller: controller, decoration: const InputDecoration(labelText: 'Name')),
                                              actions: [
                                                TextButton(onPressed: () => Navigator.of(dctx).pop(null), child: const Text('Cancel')),
                                                TextButton(onPressed: () => Navigator.of(dctx).pop(controller.text.trim()), child: const Text('Rename')),
                                              ],
                                            ),
                                          );
                                          if (!mounted) return;
                                          if (result != null && result.isNotEmpty) {
                                            await ridesProv.renameRide(ride.id, result);
                                            if (!mounted) return;
                                            messenger.showSnackBar(SnackBar(content: Text('Ride renamed'), duration: Duration(seconds: 2)));
                                          }
                                          setState(() => _revealedRideId = null);
                                        },
                                        child: Container(
                                          color: Colors.blueGrey,
                                          alignment: Alignment.center,
                                          child: Column(
                                            mainAxisSize: MainAxisSize.min,
                                            children: const [
                                              Icon(Icons.edit, color: Colors.white),
                                              SizedBox(height: 4),
                                              Text('Rename', style: TextStyle(color: Colors.white)),
                                            ],
                                          ),
                                        ),
                                      ),
                                    ),
                                  ],
                                ),
                              ),
                            ),
                          ),
                        ),
                      ),
                    ),
                  ],
                ),
              ),
            ),
          );
        },
      ),
    );
  }
}


class _StatItem extends StatelessWidget {
  final IconData icon;
  final String label;
  final String value;

  const _StatItem({required this.icon, required this.label, required this.value});

  @override
  Widget build(BuildContext context) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Row(
          children: [
            Icon(icon, size: 18, color: Theme.of(context).colorScheme.onSurfaceVariant),
            const SizedBox(width: 8),
            Text(label, style: Theme.of(context).textTheme.bodySmall),
          ],
        ),
        const SizedBox(height: 4),
        Text(value, style: Theme.of(context).textTheme.bodyMedium?.copyWith(fontWeight: FontWeight.w600)),
      ],
    );
  }
}
