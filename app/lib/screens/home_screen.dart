import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:intl/intl.dart';
import '../models/sled.dart';
import '../data/sleds.dart';
import '../data/tracks.dart';
import '../widgets/track_card.dart';
import 'track_detail_screen.dart';
import '../providers/rides_provider.dart';
import '../widgets/sled_card.dart';
import 'day_detail_screen.dart';
import 'settings_screen.dart';
import 'sled_detail_screen.dart';

class HomeScreen extends StatefulWidget {
  const HomeScreen({super.key});

  @override
  State<HomeScreen> createState() => _HomeScreenState();
}

class _HomeScreenState extends State<HomeScreen> {
  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addPostFrameCallback((_) {
      // Capture provider synchronously to avoid using BuildContext across
      // the async delay, then use the captured reference inside the delay.
      final ridesProv = context.read<RidesProvider>();
      Future.delayed(const Duration(milliseconds: 200), () {
        if (ridesProv.rides.isEmpty) {
          ridesProv.loadMockData();
        }
      });
    });
    _loadFavorites();
  }

  double gx = 0.0;
  double gy = 0.0;

  int _selectedIndex = 0;

  void _onItemTapped(int index) {
    setState(() {
      _selectedIndex = index;
    });
  }

  final Set<String> _favoriteIds = {};

  Future<void> _loadFavorites() async {
    try {
      final prefs = await SharedPreferences.getInstance();
      final favs = prefs.getStringList('favorite_sleds') ?? <String>[];
      setState(() {
        _favoriteIds.clear();
        _favoriteIds.addAll(favs);
      });
    } catch (e, st) {
      // If platform channel isn't ready or another error occurs, log and continue with empty favorites
      // ignore: avoid_print
      print('Failed to load favorites: $e\n$st');
    }
  }

  Future<void> _toggleFavorite(String sledId) async {
    final prefs = await SharedPreferences.getInstance();
    setState(() {
      if (_favoriteIds.contains(sledId)) {
        _favoriteIds.remove(sledId);
      } else {
        _favoriteIds.add(sledId);
      }
    });
    await prefs.setStringList('favorite_sleds', _favoriteIds.toList());
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Rodl'),
        centerTitle: true,
        actions: [
          IconButton(
            icon: const Icon(Icons.settings),
            onPressed: () {
              Navigator.push(
                context,
                MaterialPageRoute(builder: (_) => const SettingsScreen()),
              );
            },
          ),
        ],
      ),
      bottomNavigationBar: BottomNavigationBar(
        currentIndex: _selectedIndex,
        onTap: _onItemTapped,
        items: const [
          BottomNavigationBarItem(
            icon: Icon(Icons.sledding),
            label: "Rodls",
          ),
          BottomNavigationBarItem(
            icon: Icon(Icons.landscape),
            label: "Rides",
          ),
          BottomNavigationBarItem(
            icon: Icon(Icons.gesture),
            label: "Tracks",
          ),
        ],
      ),
      body: Padding(
        padding: const EdgeInsets.all(16.0),
        child: _buildSelectedTab(),
      ),
    );
  }

  Widget _buildSelectedTab() {
    switch (_selectedIndex) {
      case 0:
        return _buildRodls();
      case 1:
        return _buildRides();
      case 2:
        return _buildTracks();
      default:
        return const SizedBox();
    }
  }

  Widget _buildTracks() {
    final tracks = baseTracks;

    return Scaffold(
      body: Column(
        children: [
          Expanded(
            child: ListView.builder(
              itemCount: tracks.length,
              itemBuilder: (context, index) {
                final t = tracks[index];
                return TrackCard(
                  track: t,
                  onDetails: () {
                    Navigator.push(
                      context,
                      MaterialPageRoute(builder: (_) => TrackDetailScreen(track: t)),
                    );
                  },
                );
              },
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildRodls() {
    final sleds = baseSleds
        .map((s) => Sled(
              id: s.id,
              name: s.name,
              imagePath: s.imagePath,
              primaryColor: s.primaryColor,
              secondaryColor: s.secondaryColor,
              isFavorite: _favoriteIds.contains(s.id),
            ))
        .toList();

    sleds.sort((a, b) => (b.isFavorite ? 1 : 0) - (a.isFavorite ? 1 : 0));

    return Scaffold(
      body: Column(
        children: [
          Expanded(
            child: ListView.builder(
              itemCount: sleds.length,
              itemBuilder: (context, index) {
                return SledCard(
                  sled: sleds[index],
                  onTap: () {
                    Navigator.push(
                      context,
                      MaterialPageRoute(
                        builder: (_) => SledDetailScreen(sled: sleds[index]),
                      ),
                    );
                  },
                  onFavoriteToggle: () => _toggleFavorite(sleds[index].id),
                );
              },
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildRides() {
    return Scaffold(
      body: Consumer<RidesProvider>(
        builder: (context, ridesProvider, child) {
          final ridesByDay = ridesProvider.ridesByDay;
          final sortedDates = ridesByDay.keys.toList()
            ..sort((a, b) => b.compareTo(a));

          if (sortedDates.isEmpty) {
            return const Center(child: Text('No rides recorded yet'));
          }

          return ListView.builder(
            padding: const EdgeInsets.all(16),
            itemCount: sortedDates.length,
            itemBuilder: (context, index) {
              final date = sortedDates[index];
              final rides = ridesByDay[date]!;
              final totalDuration = rides.fold<int>(
                0,
                (sum, ride) => sum + ride.durationSeconds,
              );
              final maxSpeed = rides
                  .map((r) => r.maxSpeed)
                  .reduce((a, b) => a > b ? a : b);

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
                        builder: (_) => DayDetailScreen(date: date),
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
                            Text(
                              DateFormat('EEEE, MMM d').format(date),
                              style: Theme.of(context).textTheme.titleMedium
                                  ?.copyWith(fontWeight: FontWeight.bold),
                            ),
                            Container(
                              padding: const EdgeInsets.symmetric(
                                horizontal: 12,
                                vertical: 4,
                              ),
                              decoration: BoxDecoration(
                                color: Theme.of(
                                  context,
                                ).colorScheme.primaryContainer,
                                borderRadius: BorderRadius.circular(12),
                              ),
                              child: Text(
                                '${rides.length} ride${rides.length > 1 ? 's' : ''}',
                                style: TextStyle(
                                  color: Theme.of(
                                    context,
                                  ).colorScheme.onPrimaryContainer,
                                  fontWeight: FontWeight.w500,
                                ),
                              ),
                            ),
                          ],
                        ),
                        const SizedBox(height: 12),
                        Row(
                          children: [
                            _StatChip(
                              icon: Icons.timer_outlined,
                              label: _formatDuration(totalDuration),
                            ),
                            const SizedBox(width: 16),
                            _StatChip(
                              icon: Icons.speed,
                              label: '${maxSpeed.toStringAsFixed(1)} km/h',
                            ),
                          ],
                        ),
                      ],
                    ),
                  ),
                ),
              );
            },
          );
        },
      ),    
    );
  }

  String _formatDuration(int seconds) {
    final hours = seconds ~/ 3600;
    final minutes = (seconds % 3600) ~/ 60;
    if (hours > 0) {
      return '${hours}h ${minutes}m';
    }
    return '${minutes}m';
  }
}

class _StatChip extends StatelessWidget {
  final IconData icon;
  final String label;

  const _StatChip({required this.icon, required this.label});

  @override
  Widget build(BuildContext context) {
    return Row(
      mainAxisSize: MainAxisSize.min,
      children: [
        Icon(icon, size: 16, color: Theme.of(context).colorScheme.outline),
        const SizedBox(width: 4),
        Text(
          label,
          style: TextStyle(
            color: Theme.of(context).colorScheme.outline,
            fontSize: 13,
          ),
        ),
      ],
    );
  }
}
