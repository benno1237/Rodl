import 'package:flutter/material.dart';
import 'dart:async';

import 'package:provider/provider.dart';
import '../providers/settings_provider.dart';
import '../providers/rides_provider.dart';
import '../services/tile_cache.dart';

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
              _buildSectionHeader('Tile Cache'),
              const SizedBox(height: 12),
              _TileCacheCard(),
              const SizedBox(height: 24),
              _buildSectionHeader('Demo Data'),
              const SizedBox(height: 12),
              Card(
                elevation: 2,
                shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
                child: Padding(
                  padding: const EdgeInsets.all(12),
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Consumer<SettingsProvider>(
                        builder: (ctx, prov, child) {
                          return Column(
                            children: [
                              SwitchListTile(
                                title: const Text('Enable mock rides'),
                                subtitle: const Text('Generate demo rides for testing'),
                                value: prov.mockRidesEnabled,
                                onChanged: (v) async {
                                  final ridesProv = ctx.read<RidesProvider>();
                                  await prov.setMockRidesEnabled(v);
                                  try {
                                    if (v) {
                                      ridesProv.loadMockData();
                                    } else {
                                      await ridesProv.removeMockRides();
                                    }
                                  } catch (_) {}
                                },
                              ),
                              const SizedBox(height: 8),
                              ElevatedButton(
                                onPressed: () async {
                                  final ridesProv = ctx.read<RidesProvider>();
                                  final messenger = ScaffoldMessenger.of(ctx);
                                  messenger.showSnackBar(const SnackBar(content: Text('Analyzing rides...')));
                                  try {
                                    await ridesProv.analyzeAndMatchAll();
                                    if (!mounted) return;
                                    messenger.showSnackBar(const SnackBar(content: Text('Ride analysis complete')));
                                  } catch (e) {
                                    if (!mounted) return;
                                    messenger.showSnackBar(SnackBar(content: Text('Analysis failed: $e')));
                                  }
                                },
                                child: const Text('Analyze stored rides'),
                              ),
                              const SizedBox(height: 8),
                              ElevatedButton(
                                onPressed: () async {
                                  final ridesProv = ctx.read<RidesProvider>();
                                  final messenger = ScaffoldMessenger.of(ctx);
                                  final confirmed = await showDialog<bool>(
                                    context: ctx,
                                    builder: (dctx) => AlertDialog(
                                      title: const Text('Remove all rides'),
                                      content: const Text('This will permanently delete all stored rides. Continue?'),
                                      actions: [
                                        TextButton(onPressed: () => Navigator.of(dctx).pop(false), child: const Text('Cancel')),
                                        TextButton(onPressed: () => Navigator.of(dctx).pop(true), child: const Text('Delete', style: TextStyle(color: Colors.red))),
                                      ],
                                    ),
                                  );
                                  if (confirmed != true) return;
                                  messenger.showSnackBar(const SnackBar(content: Text('Removing all rides...')));
                                  try {
                                    await ridesProv.removeAllRides();
                                    if (!mounted) return;
                                    messenger.showSnackBar(const SnackBar(content: Text('All rides removed')));
                                  } catch (e) {
                                    if (!mounted) return;
                                    messenger.showSnackBar(SnackBar(content: Text('Remove failed: $e')));
                                  }
                                },
                                child: const Text('Remove all rides'),
                              ),
                              const SizedBox(height: 8),
                              ElevatedButton(
                                onPressed: () async {
                                  final ridesProv = ctx.read<RidesProvider>();
                                  final messenger = ScaffoldMessenger.of(ctx);
                                  messenger.showSnackBar(const SnackBar(content: Text('Importing GPX files...')));
                                  try {
                                    final cnt = await ridesProv.importGpxFromAppFiles();
                                    if (!mounted) return;
                                    messenger.showSnackBar(SnackBar(content: Text('Imported $cnt GPX file(s)')));
                                  } catch (e) {
                                    if (!mounted) return;
                                    messenger.showSnackBar(SnackBar(content: Text('Import failed: $e')));
                                  }
                                },
                                child: const Text('Import GPX from app files'),
                              ),
                              SwitchListTile(
                                title: const Text('Enable mock leaderboard'),
                                subtitle: const Text('Show demo leaderboard entries'),
                                value: prov.mockLeaderboardEnabled,
                                onChanged: (v) async {
                                  await prov.setMockLeaderboardEnabled(v);
                                },
                              ),
                            ],
                          );
                        },
                      ),
                    ],
                  ),
                ),
              ),
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
