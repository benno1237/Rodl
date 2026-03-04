import 'dart:convert';
import 'package:flutter/material.dart';

class Leaderboard extends StatefulWidget {
  final List<String> sections;
  final int selectedIndex; // index into sections list

  const Leaderboard({super.key, required this.sections, required this.selectedIndex});

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
    // Render as an expandable card so the leaderboard can collapse/expand.
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

        final entries = _dummyEntriesFor(section);

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

                          return RichText(
                            text: TextSpan(
                              style: Theme.of(ctx).textTheme.bodyLarge?.copyWith(color: Theme.of(ctx).colorScheme.onSurface),
                              children: [
                                TextSpan(text: entries[i]['name'] as String),
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
  }
}
