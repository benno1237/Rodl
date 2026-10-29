import 'dart:async';

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
import '../providers/settings_provider.dart';
import '../services/ble_service.dart';
import '../services/name_cache.dart';
import '../widgets/sled_card.dart';
import 'day_detail_screen.dart';
import 'settings_screen.dart';
import 'sled_detail_screen.dart';

/// A sled from `sleds.json` combined with its live discovery state.
class _SledEntry {
  final Sled sled;
  final String? mac;
  final DiscoveredBoard? board;
  final bool online;
  final bool mine;

  _SledEntry({
    required this.sled,
    required this.mac,
    required this.board,
    required this.online,
    required this.mine,
  });

  bool get inUse => board?.inUse == true && !mine;
}

class HomeScreen extends StatefulWidget {
  const HomeScreen({super.key});

  @override
  State<HomeScreen> createState() => _HomeScreenState();
}

class _HomeScreenState extends State<HomeScreen> {
  // Discovery state.
  List<DiscoveredBoard> _discovered = const [];
  bool _scanning = false;
  bool _btPromptShowing = false;
  StreamSubscription<List<DiscoveredBoard>>? _discSub;
  StreamSubscription<bool>? _discStateSub;
  String? _connectingSledId;

  @override
  void initState() {
    super.initState();
    _loadFavorites();
    WidgetsBinding.instance.addPostFrameCallback((_) {
      if (!mounted) return;
      final ble = context.read<SettingsProvider>().bleService;
      _discSub = ble.discoveryStream.listen((boards) {
        if (!mounted) return;
        setState(() => _discovered = boards);
        _cacheNames(boards);
      });
      _discStateSub = ble.discoveryStateStream.listen((scanning) {
        if (!mounted) return;
        setState(() => _scanning = scanning);
      });
      NameCache.ensureLoaded().then((_) {
        if (mounted) setState(() {});
      });
      _startDiscovery();
    });
  }

  @override
  void dispose() {
    _discSub?.cancel();
    _discStateSub?.cancel();
    super.dispose();
  }

  double gx = 0.0;
  double gy = 0.0;

  int _selectedIndex = 0;

  void _onItemTapped(int index) {
    final previous = _selectedIndex;
    setState(() {
      _selectedIndex = index;
    });
    if (index == 0 && previous != 0) {
      // Re-entering the Rodls tab: refresh discovery.
      _startDiscovery();
    } else if (index != 0) {
      context.read<SettingsProvider>().bleService.stopDiscovery();
    }
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

  // ---- Discovery ----------------------------------------------------------

  Future<void> _startDiscovery() async {
    if (_scanning || _btPromptShowing || !mounted) return;
    final ble = context.read<SettingsProvider>().bleService;
    if (!await ble.isAdapterOn) {
      // Ask before firing the system enable dialog so the user knows why.
      await _promptEnableBluetooth(ble);
      return;
    }
    final ok = await ble.startDiscovery();
    if (!ok && mounted) {
      final error = ble.lastError;
      if (error != null) {
        ScaffoldMessenger.of(context)
            .showSnackBar(SnackBar(content: Text(error)));
      }
    }
  }

  /// Explain why Bluetooth is needed, then let the user turn it on.
  Future<void> _promptEnableBluetooth(BleService ble) async {
    if (_btPromptShowing || !mounted) return;
    _btPromptShowing = true;
    try {
      final enable = await showDialog<bool>(
        context: context,
        builder: (ctx) => AlertDialog(
          title: const Text('Bluetooth is off'),
          content: const Text(
            'Rodl uses Bluetooth to discover and connect to your sleds.',
          ),
          actions: [
            TextButton(
              onPressed: () => Navigator.of(ctx).pop(false),
              child: const Text('Not now'),
            ),
            FilledButton(
              onPressed: () => Navigator.of(ctx).pop(true),
              child: const Text('Enable Bluetooth'),
            ),
          ],
        ),
      );
      if (enable != true || !mounted) return;
      final ok = await ble.ensureAdapterOn();
      if (!mounted) return;
      if (ok) {
        await _startDiscovery();
      } else {
        ScaffoldMessenger.of(context).showSnackBar(SnackBar(
          content: Text(ble.lastError ??
              'Bluetooth is still off. Turn it on in system settings.'),
        ));
      }
    } finally {
      _btPromptShowing = false;
    }
  }

  void _cacheNames(List<DiscoveredBoard> boards) {
    for (final b in boards) {
      if (b.name.isNotEmpty && NameCache.nameFor(b.remoteId) != b.name) {
        NameCache.set(b.remoteId, b.name);
      }
    }
  }

  String _shortMac(String mac) =>
      mac.length > 8 ? mac.substring(mac.length - 8) : mac;

  // ---- Navigation / connection -------------------------------------------

  Future<void> _openDetail(Sled sled) async {
    await Navigator.push(
      context,
      MaterialPageRoute(builder: (_) => SledDetailScreen(sled: sled)),
    );
  }

  Future<void> _connectAndOpen(Sled sled, String mac) async {
    if (_connectingSledId != null) return;
    setState(() => _connectingSledId = sled.id);
    final provider = context.read<SettingsProvider>();
    bool ok = false;
    try {
      ok = await provider.connectToSled(sled.id, address: mac);
    } finally {
      if (mounted) setState(() => _connectingSledId = null);
    }
    if (!mounted) return;
    if (ok) {
      await _openDetail(sled);
    } else {
      final error =
          provider.bleService.lastError ?? 'Could not connect to the sled.';
      ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text(error)));
    }
  }

  Future<void> _adoptBoard(DiscoveredBoard board) async {
    if (board.inUse) {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(content: Text('In use by another user.')),
      );
      return;
    }
    final provider = context.read<SettingsProvider>();
    final nameController = TextEditingController(
      text: board.name.toUpperCase().startsWith('RODL-') ? '' : board.name,
    );
    String? chosenImage;
    String? adoptedSledId;
    String? adoptError;
    bool saving = false;

    final pictures = <String>[];
    for (final s in baseSleds) {
      if (!pictures.contains(s.imagePath)) pictures.add(s.imagePath);
    }

    final confirmed = await showModalBottomSheet<bool>(
      context: context,
      isScrollControlled: true,
      builder: (ctx) {
        return StatefulBuilder(builder: (ctx, setSheetState) {
          return Padding(
            padding: EdgeInsets.only(
              bottom: MediaQuery.of(ctx).viewInsets.bottom,
            ),
            child: Padding(
              padding: const EdgeInsets.all(16),
              child: Column(
                mainAxisSize: MainAxisSize.min,
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text(
                    'Adopt this sled',
                    style: Theme.of(ctx)
                        .textTheme
                        .titleMedium
                        ?.copyWith(fontWeight: FontWeight.bold),
                  ),
                  const SizedBox(height: 2),
                  Text(
                    'Board ${_shortMac(board.remoteId)}',
                    style: const TextStyle(fontSize: 12, color: Colors.grey),
                  ),
                  const SizedBox(height: 16),
                  TextField(
                    controller: nameController,
                    maxLength: BleService.maxNameLength,
                    decoration: const InputDecoration(
                      border: OutlineInputBorder(),
                      labelText: 'Display name (Bluetooth name)',
                      hintText: 'e.g. Red Lightning',
                    ),
                  ),
                  const SizedBox(height: 8),
                  Text(
                    'Picture',
                    style: Theme.of(ctx).textTheme.titleSmall,
                  ),
                  const SizedBox(height: 8),
                  Wrap(
                    spacing: 12,
                    runSpacing: 8,
                    children: [
                      for (final pic in pictures)
                        GestureDetector(
                          onTap: () => setSheetState(() => chosenImage = pic),
                          child: Container(
                            decoration: BoxDecoration(
                              borderRadius: BorderRadius.circular(10),
                              border: Border.all(
                                color: chosenImage == pic
                                    ? Theme.of(ctx).colorScheme.primary
                                    : Colors.transparent,
                                width: 3,
                              ),
                            ),
                            child: ClipRRect(
                              borderRadius: BorderRadius.circular(7),
                              child: Image.asset(
                                pic,
                                width: 96,
                                height: 64,
                                fit: BoxFit.cover,
                              ),
                            ),
                          ),
                        ),
                    ],
                  ),
                  const SizedBox(height: 12),
                  if (adoptError != null)
                    Padding(
                      padding: const EdgeInsets.only(bottom: 8),
                      child: Text(
                        adoptError!,
                        style: const TextStyle(color: Colors.red, fontSize: 13),
                      ),
                    ),
                  SizedBox(
                    width: double.infinity,
                    child: FilledButton(
                      onPressed: saving
                          ? null
                          : () async {
                              final name = nameController.text.trim();
                              final pic = chosenImage;
                              if (name.isEmpty) {
                                setSheetState(
                                    () => adoptError = 'Enter a name.');
                                return;
                              }
                              if (pic == null) {
                                setSheetState(() =>
                                    adoptError = 'Pick a picture.');
                                return;
                              }
                              Sled? entry;
                              for (final s in baseSleds) {
                                if (s.imagePath == pic) {
                                  entry = s;
                                  break;
                                }
                              }
                              if (entry == null) {
                                setSheetState(() => adoptError =
                                    'This picture has no sled entry.');
                                return;
                              }
                              setSheetState(() {
                                saving = true;
                                adoptError = null;
                              });
                              final ok = await provider.connectToSled(
                                entry.id,
                                address: board.remoteId,
                              );
                              if (!ok) {
                                setSheetState(() {
                                  saving = false;
                                  adoptError = provider
                                          .bleService.lastError ??
                                      'Could not connect to the board.';
                                });
                                return;
                              }
                              await provider.bleService.writeDeviceName(name);
                              await NameCache.set(board.remoteId, name);
                              adoptedSledId = entry.id;
                              if (mounted) setState(() {});
                              if (ctx.mounted) Navigator.of(ctx).pop(true);
                            },
                      child: saving
                          ? const SizedBox(
                              width: 20,
                              height: 20,
                              child: CircularProgressIndicator(
                                  strokeWidth: 2.5),
                            )
                          : const Text('Adopt'),
                    ),
                  ),
                ],
              ),
            ),
          );
        });
      },
    );

    if (confirmed == true && mounted && adoptedSledId != null) {
      final entry = sledsById[adoptedSledId!];
      if (entry != null) {
        await _openDetail(Sled(
          id: entry.id,
          name: nameController.text.trim(),
          imagePath: entry.imagePath,
          primaryColor: entry.primaryColor,
          secondaryColor: entry.secondaryColor,
        ));
      }
    }
    nameController.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Rodl'),
        centerTitle: true,
        actions: [
          IconButton(
            tooltip: 'Scan for sleds',
            icon: _scanning
                ? const SizedBox(
                    width: 20,
                    height: 20,
                    child: CircularProgressIndicator(strokeWidth: 2),
                  )
                : const Icon(Icons.radar),
            onPressed: _scanning ? null : _startDiscovery,
          ),
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
    final provider = context.watch<SettingsProvider>();
    final activeMac =
        provider.isConnected ? provider.bleService.remoteId : null;

    final byMac = {for (final d in _discovered) d.remoteId: d};
    final pairedMacs = provider.pairedMacs;

    // Known sleds (from sleds.json) with their live discovery state.
    final entries = <_SledEntry>[];
    for (final s in baseSleds) {
      final mac = provider.pairedAddressFor(s.id);
      final board = mac != null ? byMac[mac] : null;
      final mine = mac != null && activeMac != null && mac == activeMac;
      entries.add(_SledEntry(
        sled: s,
        mac: mac,
        board: board,
        online: board != null || mine,
        mine: mine,
      ));
    }

    // Discovered boards that are not linked to any sled yet.
    final unknown = <DiscoveredBoard>[];
    for (final d in _discovered) {
      if (!pairedMacs.contains(d.remoteId)) unknown.add(d);
    }

    // Discovered sleds first (strongest signal first), then favorites,
    // then the rest - which stay greyed out until discovered.
    entries.sort((a, b) {
      if (a.online != b.online) return a.online ? -1 : 1;
      if (a.online && b.online) {
        final r = (b.board?.rssi ?? -999) - (a.board?.rssi ?? -999);
        if (r != 0) return r;
      }
      final af = _favoriteIds.contains(a.sled.id) ? 1 : 0;
      final bf = _favoriteIds.contains(b.sled.id) ? 1 : 0;
      if (af != bf) return bf - af;
      return 0;
    });

    return Scaffold(
      body: Column(
        children: [
          if (_scanning) const LinearProgressIndicator(minHeight: 2),
          if (!_scanning && _discovered.isEmpty)
            const Padding(
              padding: EdgeInsets.only(bottom: 8),
              child: Text(
                'No sleds nearby - tap the radar to scan',
                style: TextStyle(fontSize: 12, color: Colors.grey),
              ),
            ),
          Expanded(
            child: ListView.builder(
              itemCount: unknown.length + entries.length,
              itemBuilder: (context, index) {
                if (index < unknown.length) {
                  return _buildUnknownCard(unknown[index]);
                }
                return _buildSledCard(
                    provider, entries[index - unknown.length]);
              },
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildSledCard(SettingsProvider provider, _SledEntry entry) {
    final board = entry.board;
    // Prefer the cached device name: it is updated at adoption and on every
    // scan, so it is never staler than the scan entry and survives renames
    // before the next scan window.
    final name = (entry.mac != null ? NameCache.nameFor(entry.mac!) : null) ??
        (board?.name.isNotEmpty == true ? board!.name : null) ??
        entry.sled.name;

    final displaySled = Sled(
      id: entry.sled.id,
      name: name,
      imagePath: entry.sled.imagePath,
      primaryColor: entry.sled.primaryColor,
      secondaryColor: entry.sled.secondaryColor,
      isFavorite: _favoriteIds.contains(entry.sled.id),
    );

    return SledCard(
      sled: displaySled,
      dimmed: !entry.online,
      online: entry.online,
      inUse: entry.inUse,
      connecting: _connectingSledId == entry.sled.id,
      onTap: () {
        if (!entry.online) return; // greyed out sleds can not be opened
        if (entry.inUse) {
          ScaffoldMessenger.of(context).showSnackBar(
            const SnackBar(content: Text('In use by another user.')),
          );
          return;
        }
        if (entry.mine) {
          _openDetail(displaySled);
          return;
        }
        _connectAndOpen(displaySled, entry.mac!);
      },
      onFavoriteToggle: () => _toggleFavorite(entry.sled.id),
    );
  }

  Widget _buildUnknownCard(DiscoveredBoard board) {
    final named = !board.name.toUpperCase().startsWith('RODL-');
    return Card(
      margin: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
      color: Theme.of(context).colorScheme.secondaryContainer,
      child: InkWell(
        borderRadius: BorderRadius.circular(12),
        onTap: _scanning || _connectingSledId != null
            ? null
            : () => _adoptBoard(board),
        child: Padding(
          padding: const EdgeInsets.all(16),
          child: Row(
            children: [
              CircleAvatar(
                backgroundColor:
                    Theme.of(context).colorScheme.primaryContainer,
                child: const Icon(Icons.sledding),
              ),
              const SizedBox(width: 16),
              Expanded(
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text(
                      named ? 'New sled: ${board.name}' : 'New sled found',
                      style: Theme.of(context).textTheme.titleMedium?.copyWith(
                            fontWeight: FontWeight.w600,
                          ),
                    ),
                    const SizedBox(height: 4),
                    Text(
                      board.inUse
                          ? 'In use by another user'
                          : 'Board ${_shortMac(board.remoteId)} - tap to adopt',
                      style: TextStyle(
                        fontSize: 12,
                        color: board.inUse
                            ? Colors.orange
                            : Theme.of(context).colorScheme.outline,
                      ),
                    ),
                  ],
                ),
              ),
              if (board.inUse)
                const Icon(Icons.lock, color: Colors.orange)
              else
                const Icon(Icons.add_circle_outline),
            ],
          ),
        ),
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
                final trackNames = rides.map((r) => r.trackName).where((n) => n != null).toSet();
                final String? uniqueTrack = trackNames.length == 1 ? trackNames.first : null;

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
                                color: Theme.of(context)
                                    .colorScheme
                                    .primaryContainer,
                                borderRadius: BorderRadius.circular(12),
                              ),
                              child: Text(
                                '${rides.length} ride${rides.length > 1 ? 's' : ''}',
                                style: TextStyle(
                                  color: Theme.of(context)
                                      .colorScheme
                                      .onPrimaryContainer,
                                  fontWeight: FontWeight.w500,
                                ),
                              ),
                            ),
                          ],
                        ),
                        const SizedBox(height: 6),
                        
                        Row(
                          children: [
                            if (uniqueTrack != null) ...[
                              _StatChip(icon: Icons.place, label: uniqueTrack),
                              const SizedBox(width: 16),
                            ],
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
