import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:intl/intl.dart';
import '../providers/rides_provider.dart';
import 'ride_detail_screen.dart';

class DayDetailScreen extends StatelessWidget {
  final DateTime date;

  const DayDetailScreen({super.key, required this.date});

  @override
  Widget build(BuildContext context) {
    final rides = context.read<RidesProvider>().getRidesForDay(date);

    return Scaffold(
      appBar: AppBar(
        title: Text(DateFormat('MMM d, yyyy').format(date)),
        centerTitle: true,
      ),
      body: ListView.builder(
        padding: const EdgeInsets.all(16),
        itemCount: rides.length,
        itemBuilder: (context, index) {
          final ride = rides[index];
          final startTime = DateFormat('HH:mm').format(ride.startTime);

          return Card(
            margin: const EdgeInsets.only(bottom: 12),
            elevation: 2,
            shape: RoundedRectangleBorder(
              borderRadius: BorderRadius.circular(16),
            ),
            child: InkWell(
              borderRadius: BorderRadius.circular(16),
              onTap: () {
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
                              'Ride ${index + 1}',
                              style: Theme.of(context).textTheme.titleMedium
                                  ?.copyWith(fontWeight: FontWeight.bold),
                            ),
                          ],
                        ),
                        Text(
                          startTime,
                          style: TextStyle(
                            color: Theme.of(context).colorScheme.outline,
                          ),
                        ),
                      ],
                    ),
                    const SizedBox(height: 12),
                    Row(
                      children: [
                        _StatItem(
                          icon: Icons.timer_outlined,
                          label: 'Duration',
                          value: ride.formattedDuration,
                        ),
                        const SizedBox(width: 24),
                        _StatItem(
                          icon: Icons.speed,
                          label: 'Max Speed',
                          value: '${ride.maxSpeed.toStringAsFixed(1)} km/h',
                        ),
                        const SizedBox(width: 24),
                        _StatItem(
                          icon: Icons.straighten,
                          label: 'Distance',
                          value:
                              '${(ride.totalDistanceKm * 1000).toStringAsFixed(0)} m',
                        ),
                      ],
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

  const _StatItem({
    required this.icon,
    required this.label,
    required this.value,
  });

  @override
  Widget build(BuildContext context) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Row(
          mainAxisSize: MainAxisSize.min,
          children: [
            Icon(icon, size: 14, color: Theme.of(context).colorScheme.outline),
            const SizedBox(width: 4),
            Text(
              label,
              style: TextStyle(
                fontSize: 11,
                color: Theme.of(context).colorScheme.outline,
              ),
            ),
          ],
        ),
        const SizedBox(height: 2),
        Text(value, style: const TextStyle(fontWeight: FontWeight.w600)),
      ],
    );
  }
}
