import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../providers/rides_provider.dart';
import '../providers/settings_provider.dart';

class Leaderboard extends StatefulWidget {
  final List<String> sections;
  final int selectedIndex; // index into sections list
  // optional filter: when provided, show only rides matched to this track
  final String? filterTrackId;
  final String? filterTrackName;

  const Leaderboard({
    super.key,
    required this.sections,
    required this.selectedIndex,
    this.filterTrackId,
    this.filterTrackName,
  });

  @override
  State<Leaderboard> createState() => _LeaderboardState();
}

class _LeaderboardState extends State<Leaderboard> {
  late int _selectedIndex;

  @override
  void initState() {
    super.initState();
    _selectedIndex = widget.selectedIndex.clamp(0, widget.sections.length - 1);
  }

  @override
  void didUpdateWidget(covariant Leaderboard oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (oldWidget.selectedIndex != widget.selectedIndex || oldWidget.sections != widget.sections) {
      _selectedIndex = widget.selectedIndex.clamp(0, widget.sections.length - 1);
    }
  }

  String _formatSeconds(int s) {
    final mins = s ~/ 60;
    final secs = s % 60;
    return '${mins.toString().padLeft(2, '0')}:${secs.toString().padLeft(2, '0')}';
  }

  String _formatDate(DateTime d) {
    const monthNames = [
      'January',
      'February',
      'March',
      'April',
      'May',
      'June',
      'July',
      'August',
      'September',
      'October',
      'November',
      'December'
    ];
    final day = d.day;
    final month = monthNames[d.month - 1];
    final year = d.year;
    return '$day $month $year';
  }

  List<Map<String, dynamic>> _dummyEntriesFor(String section) {
    final base = 300 + (section.hashCode % 120);
    final List<Map<String, dynamic>> list = List.generate(10, (i) {
      final seconds = base - (i * 5) - (section.length % 7);
      // create a fake date and a fake mean velocity based on a notional distance
      final date = DateTime.now().subtract(Duration(days: i * 3 + (section.hashCode % 5)));
      final distanceKm = 2.0 + ((section.hashCode % 5) * 0.75); // arbitrary distance per section
      final secsClamped = seconds.clamp(60, 3600);
      final hours = secsClamped / 3600.0;
      final velocityKmh = hours > 0 ? (distanceKm / hours) : 0.0;
      return {
        'name': 'Rider ${i + 1}',
        'seconds': secsClamped,
        'date': date,
        'velocity': velocityKmh,
      };
    });
    list.sort((a, b) => (a['seconds'] as int).compareTo(b['seconds'] as int));
    return list;
  }

  Color _colorFromHex(String hex) {
    var cleaned = hex.replaceAll('#', '');
    if (cleaned.length == 6) cleaned = 'FF$cleaned';
    return Color(int.parse(cleaned, radix: 16));
  }

  @override
  Widget build(BuildContext context) {
    final section = (widget.sections.isNotEmpty && _selectedIndex >= 0 && _selectedIndex < widget.sections.length)
        ? widget.sections[_selectedIndex]
        : 'Full';

    return FutureBuilder<String>(
      future: DefaultAssetBundle.of(context).loadString('assets/data/sleds.json'),
      builder: (context, snap) {
        List<dynamic> sleds = [];
        if (snap.hasData) {
          try {
            sleds = jsonDecode(snap.data!) as List<dynamic>;
          } catch (_) {
            sleds = [];
          }
        }

        return Consumer2<RidesProvider, SettingsProvider>(
          builder: (context, ridesProv, settingsProv, child) {
            final includeMock = settingsProv.mockLeaderboardEnabled;

            List<Map<String, dynamic>> entries = [];

        // If a specific track filter is provided and we're showing the "Full"
        // section, prefer real matched rides for that track. Respect the
        // mock-leaderboard toggle: when off, exclude mock rides.
        if ((widget.filterTrackId != null || widget.filterTrackName != null)) {
          final segmentIndex = _selectedIndex == 0 ? null : (_selectedIndex - 1);
          final all = ridesProv.rides.where((r) {
            final matchesFilter = (widget.filterTrackId != null && r.trackId == widget.filterTrackId) ||
                (widget.filterTrackName != null && r.trackName == widget.filterTrackName);
            final matchesSegment = segmentIndex == null ? (r.segmentIndex == null) : (r.segmentIndex == segmentIndex);
            final allowedByMockFlag = includeMock ? true : (r.isMock != true);
            return matchesFilter && matchesSegment && allowedByMockFlag;
          }).toList();

          all.sort((a, b) => a.durationSeconds.compareTo(b.durationSeconds));

          entries = all.take(10).map((r) {
            final secs = r.durationSeconds;
            final hours = secs > 0 ? (secs / 3600.0) : 0.0;
            final vel = hours > 0 ? (r.totalDistanceKm / hours) : 0.0;
            return {
              'name': r.username ?? r.name ?? 'Rider',
              'seconds': secs,
              'date': r.startTime,
              'velocity': vel,
            };
          }).toList();
        }

        // If no entries were found for the filtered track (or no filter set),
        // attempt to show best valid rides across all tracks when mocks are
        // disabled, otherwise include mock rides as well.
        if (entries.isEmpty) {
          final candidates = ridesProv.rides.where((r) => r.trackId != null && (includeMock ? true : (r.isMock != true))).toList();
          candidates.sort((a, b) => a.durationSeconds.compareTo(b.durationSeconds));
          if (candidates.isNotEmpty) {
            entries = candidates.take(10).map((r) {
              final secs = r.durationSeconds;
              final hours = secs > 0 ? (secs / 3600.0) : 0.0;
              final vel = hours > 0 ? (r.totalDistanceKm / hours) : 0.0;
              return {
                'name': r.username ?? r.name ?? 'Rider',
                'seconds': secs,
                'date': r.startTime,
                'velocity': vel,
              };
            }).toList();
          } else if (includeMock) {
            entries = _dummyEntriesFor(section);
          }
        }

        return Padding(
          padding: EdgeInsets.zero,
          child: Card(
            elevation: 2,
            clipBehavior: Clip.antiAlias,
            shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(8)),
            child: Theme(
              data: Theme.of(context).copyWith(dividerColor: Colors.transparent),
              child: ExpansionTile(
                title: Text('Leaderboard — $section', style: Theme.of(context).textTheme.titleMedium),
                initiallyExpanded: false,
                tilePadding: const EdgeInsets.symmetric(horizontal: 12, vertical: 0),
                backgroundColor: Colors.transparent,
                childrenPadding: const EdgeInsets.symmetric(horizontal: 0, vertical: 0),
                children: [
                  Container(
                    width: double.infinity,
                    decoration: BoxDecoration(
                      color: Theme.of(context).colorScheme.surface.withAlpha(30),
                      borderRadius: BorderRadius.circular(8),
                    ),
                    child: Padding(
                      padding: const EdgeInsets.all(8),
                      child: Column(
                        mainAxisSize: MainAxisSize.min,
                        children: [
                          for (var i = 0; i < entries.length; i++) ...[
                            ListTile(
                              contentPadding: EdgeInsets.zero,
                              visualDensity: const VisualDensity(vertical: -4),
                              leading: SizedBox(
                                width: 44,
                                height: 44,
                                child: Center(
                                  child: Builder(builder: (ctx) {
                                    final Color rankColor = (i == 0)
                                        ? const Color.fromARGB(255, 217, 184, 0)
                                        : (i == 1)
                                            ? const Color(0xFFC0C0C0)
                                            : (i == 2)
                                                ? const Color(0xFFCD7F32)
                                                : Theme.of(ctx).colorScheme.surface.withAlpha(30);

                                    if (i < 3) {
                                      return Stack(
                                        alignment: Alignment.center,
                                        children: [
                                          Transform.translate(
                                            offset: const Offset(0, 4), // move down 4 pixels
                                            child: Icon(
                                              Icons.emoji_events,
                                              size: 40,
                                              color: rankColor.withValues(alpha: 0.5),
                                            ),
                                          ),
                                          Text(
                                            '${i + 1}',
                                            style: const TextStyle(fontWeight: FontWeight.bold, color: Colors.black, fontSize: 18),
                                          ),
                                        ],
                                      );
                                    }

                                    return Container(
                                      padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 6),
                                      decoration: BoxDecoration(
                                        color: rankColor,
                                        borderRadius: BorderRadius.circular(6),
                                      ),
                                      child: Text(
                                        '${i + 1}',
                                        style: const TextStyle(fontWeight: FontWeight.bold, color: Colors.black, fontSize: 18),
                                      ),
                                    );
                                  }),
                                ),
                              ),
                              title: Builder(builder: (ctx) {
                                // pick sled for this entry (round-robin)
                                final sled = (sleds.isNotEmpty) ? sleds[i % sleds.length] as Map<String, dynamic> : null;
                                final sledName = sled != null ? (sled['name'] as String? ?? '') : '';
                                final sledColor = sled != null && sled['primaryColor'] is String
                                    ? _colorFromHex(sled['primaryColor'] as String)
                                    : Theme.of(ctx).colorScheme.onSurface;

                                final rawName = entries[i]['name'] as String;
                                final displayName = (settingsProv.username.isNotEmpty &&
                                        (rawName == 'Rider' || rawName.startsWith('Rider ')))
                                    ? settingsProv.username
                                    : rawName;

                                return RichText(
                                  text: TextSpan(
                                    style: Theme.of(ctx).textTheme.bodyLarge?.copyWith(color: Theme.of(ctx).colorScheme.onSurface),
                                    children: [
                                      TextSpan(text: displayName),
                                      TextSpan(text: ' / '),
                                      TextSpan(text: sledName, style: TextStyle(color: sledColor)),
                                    ],
                                  ),
                                );
                              }),
                              subtitle: Text(_formatDate(entries[i]['date'] as DateTime)),
                              trailing: Column(
                                mainAxisSize: MainAxisSize.min,
                                crossAxisAlignment: CrossAxisAlignment.end,
                                children: [
                                  Text(_formatSeconds(entries[i]['seconds'] as int),
                                      style: Theme.of(context).textTheme.bodyLarge?.copyWith(fontFeatures: const [FontFeature.tabularFigures()])),
                                  Text(
                                    '${(entries[i]['velocity'] as double).toStringAsFixed(1)} km/h',
                                    style: Theme.of(context).textTheme.bodySmall,
                                  ),
                                ],
                              ),
                            ),
                            if (i != entries.length - 1)
                              Divider(height: 2, thickness: 1, color: Colors.black.withValues(alpha: .08)),
                          ],
                        ],
                      ),
                    ),
                  ),
                ],
              ),
            ),
          ),
          );
        },
        );
      },
    );
  }
}
