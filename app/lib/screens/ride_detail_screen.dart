import 'package:flutter/material.dart';
import 'package:flutter_map/flutter_map.dart';
import 'dart:ui' as ui;
import '../services/tile_cache.dart';
import 'package:latlong2/latlong.dart';
import 'package:intl/intl.dart';
import 'dart:math' as math;
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
  final double _initialMapZoom = 14.0;
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
                  initialZoom: _initialMapZoom,
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
                      Polyline(
                        points: points.map((p) => LatLng(p.lat, p.lon)).toList(),
                        color: Theme.of(context).colorScheme.primary,
                        strokeWidth: 4,
                      ),
                    ],
                  ),
                  if (_selectedPointIndex != null)
                    MarkerLayer(
                      markers: [
                        Marker(
                          point: LatLng(points[_selectedPointIndex!].lat, points[_selectedPointIndex!].lon),
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
                  const SizedBox(height: 12),
                  _buildSpeedProfile(points),
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
    if (points.isEmpty) {
      return Padding(
        padding: const EdgeInsets.symmetric(vertical: 8),
        child: Text('No points available', style: Theme.of(context).textTheme.bodyMedium),
      );
    }

    return Card(
      elevation: 2,
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(8)),
      child: Theme(
        data: Theme.of(context).copyWith(dividerColor: Colors.transparent),
        child: ExpansionTile(
          title: Text('Points — ${points.length}', style: Theme.of(context).textTheme.titleMedium),
          initiallyExpanded: false,
          childrenPadding: const EdgeInsets.symmetric(vertical: 0),
          children: [
            Container(
              width: double.infinity,
              decoration: BoxDecoration(
                color: Theme.of(context).colorScheme.surface.withAlpha(30),
                borderRadius: BorderRadius.circular(8),
              ),
              child: Column(
                mainAxisSize: MainAxisSize.min,
                children: List<Widget>.generate(points.length * 2 - 1, (i) {
                  if (i.isOdd) {
                    return Divider(height: 1, thickness: 1, color: Theme.of(context).colorScheme.onSurface.withAlpha(15));
                  }

                  final index = i ~/ 2;
                  final point = points[index];
                  final isSelected = _selectedPointIndex == index;

                  return InkWell(
                    onTap: () {
                      setState(() {
                        _selectedPointIndex = isSelected ? null : index;
                      });
                      if (!isSelected) {
                        _mapController.move(LatLng(point.lat, point.lon), 16);
                      }
                    },
                    child: Container(
                      color: isSelected ? Theme.of(context).colorScheme.primary.withAlpha(31) : Colors.transparent,
                      padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 10),
                      child: Row(
                        children: [
                          
                          Expanded(
                            child: Column(
                              crossAxisAlignment: CrossAxisAlignment.start,
                              children: [
                                Text('Point ${index + 1}', style: const TextStyle(fontWeight: FontWeight.w600)),
                                const SizedBox(height: 4),
                                Row(
                                  children: [
                                    _MiniStat(icon: Icons.speed, value: '${point.speedKmh.toStringAsFixed(1)} km/h'),
                                    const SizedBox(width: 12),
                                    _MiniStat(icon: Icons.vibration, value: '${point.acceleration.toStringAsFixed(2)} g'),
                                  ],
                                ),
                              ],
                            ),
                          ),
                          
                        ],
                      ),
                    ),
                  );
                }),
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildSpeedProfile(List<GPSPoint> points) {
    final speeds = points.map((p) => p.speedKmh).toList();
    if (speeds.isEmpty) {
      return Padding(
        padding: const EdgeInsets.symmetric(vertical: 8),
        child: Text('No speed data', style: Theme.of(context).textTheme.bodyMedium),
      );
    }

    return Card(
      elevation: 2,
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(8)),
      child: Padding(
        padding: const EdgeInsets.all(12),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text('Speed Profile', style: Theme.of(context).textTheme.titleMedium?.copyWith(fontWeight: FontWeight.bold)),
            const SizedBox(height: 8),
            SizedBox(
              height: 240,
              child: _SpeedProfile(
                points: points,
                onIndexChanged: (idx) {
                  setState(() {
                    _selectedPointIndex = idx;
                  });
                },
              ),
            ),
          ],
        ),
      ),
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

class SpeedProfilePainter extends CustomPainter {
  final List<double> speeds;
  final int? highlightIndex;
  final List<double>? yTicks; // absolute values for y tick/grid lines
  final List<double>? xTicks; // fractions 0..1 for vertical grid lines
  final double? meanS;
  final double minS;
  final double maxS;

  SpeedProfilePainter(this.speeds, {this.highlightIndex, this.yTicks, this.xTicks, this.meanS, required this.minS, required this.maxS});

  @override
  void paint(Canvas canvas, Size size) {
    final paintFill = Paint()
      ..style = PaintingStyle.fill
      ..color = const Color(0xFF42A5F5).withAlpha(20);

    final paintLine = Paint()
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2.0
      ..color = const Color(0xFF1E88E5);

    final paintDot = Paint()..color = const Color(0xFF1E88E5);

    final paintGrid = Paint()
      ..style = PaintingStyle.stroke
      ..strokeWidth = 1.0
      ..color = const Color.fromARGB(20, 0, 0, 0);

    final n = speeds.length;
    if (n == 0) return;

    // use provided min/max for consistent baseline
    final double minS = this.minS;
    final double maxS = this.maxS;

    final dx = size.width / (n - 1);

    final path = ui.Path();
    final fillPath = ui.Path();

    for (var i = 0; i < n; i++) {
      final x = dx * i;
      final t = (speeds[i] - minS) / (maxS - minS);
      final y = size.height - (t * size.height);
      if (i == 0) {
        path.moveTo(x, y);
        fillPath.moveTo(x, size.height);
        fillPath.lineTo(x, y);
      } else {
        path.lineTo(x, y);
        fillPath.lineTo(x, y);
      }
    }

    // close fill path down to bottom-right
    fillPath.lineTo(size.width, size.height);
    fillPath.close();

    canvas.drawPath(fillPath, paintFill);
    canvas.drawPath(path, paintLine);

    // draw horizontal grid lines based on provided yTicks (or default division)
    final ticks = yTicks ?? [for (var gi = 0; gi <= 6; gi++) minS + (gi / 6) * (maxS - minS)];
    for (final tVal in ticks) {
      final frac = (tVal - minS) / (maxS - minS);
      final gy = size.height - (frac * size.height);
      canvas.drawLine(Offset(0, gy), Offset(size.width, gy), paintGrid);
    }
    // vertical grid lines: use provided xTicks fractions if available, otherwise fall back to quartiles
    if (xTicks != null && xTicks!.isNotEmpty) {
      for (final frac in xTicks!) {
        final gx = (frac.clamp(0.0, 1.0)) * size.width;
        canvas.drawLine(Offset(gx, 0), Offset(gx, size.height), paintGrid);
      }
    } else {
      canvas.drawLine(Offset(0, 0), Offset(0, size.height), paintGrid);
      canvas.drawLine(Offset(size.width / 4, 0), Offset(size.width / 4, size.height), paintGrid);
      canvas.drawLine(Offset(size.width / 2, 0), Offset(size.width / 2, size.height), paintGrid);
      canvas.drawLine(Offset(size.width * 3 / 4, 0), Offset(size.width * 3 / 4, size.height), paintGrid);
      canvas.drawLine(Offset(size.width, 0), Offset(size.width, size.height), paintGrid);
    }

    // draw mean speed as dashed black horizontal line (if provided and within range)
    if (meanS != null && meanS! >= minS && meanS! <= maxS) {
      final fracMean = (meanS! - minS) / (maxS - minS);
      final my = size.height - (fracMean * size.height);
      final paintMean = Paint()
        ..style = PaintingStyle.stroke
        ..strokeWidth = 1.5
        ..color = Colors.black;
      const dashW = 10.0;
      const gapW = 5.0;
      double sx = 2.5;
      while (sx < size.width) {
        final ex = math.min(sx + dashW, size.width);
        canvas.drawLine(Offset(sx, my), Offset(ex, my), paintMean);
        sx = ex + gapW;
      }
    }

    // draw small dots
    for (var i = 0; i < n; i += (n ~/ 20 == 0 ? 1 : (n ~/ 20))) {
      final x = dx * i;
      final t = (speeds[i] - minS) / (maxS - minS);
      final y = size.height - (t * size.height);
      canvas.drawCircle(Offset(x, y), 2.0, paintDot);
    }

    // draw highlighted intersection point if provided
    if (highlightIndex != null && highlightIndex! >= 0 && highlightIndex! < n) {
      final i = highlightIndex!;
      final x = dx * i;
      final t = (speeds[i] - minS) / (maxS - minS);
      final y = size.height - (t * size.height);
      final highlight = Paint()..color = const Color(0xFF1E88E5);
      canvas.drawCircle(Offset(x, y), 5.0, highlight);
      canvas.drawCircle(Offset(x, y), 8.0, highlight..color = const Color.fromARGB(30, 255, 255, 255));
    }
  }

  @override
  bool shouldRepaint(covariant SpeedProfilePainter oldDelegate) {
    if (oldDelegate.speeds.length != speeds.length) return true;
    for (var i = 0; i < speeds.length; i++) {
      if (oldDelegate.speeds[i] != speeds[i]) return true;
    }
    if (oldDelegate.meanS != meanS) return true;
    return false;
  }
}


class _SpeedProfile extends StatefulWidget {
  final List<GPSPoint> points;
  final ValueChanged<int?>? onIndexChanged;

  const _SpeedProfile({required this.points, this.onIndexChanged});

  @override
  State<_SpeedProfile> createState() => _SpeedProfileState();
}

class _SpeedProfileState extends State<_SpeedProfile> {
  double? _tapX;
  int? _selectedIndex;
  OverlayEntry? _overlayEntry;
  Offset? _overlayGlobal;
  String _overlayText = '';
  String _overlaySubtitle = '';

  void _onDrag(Offset localPosition, double width, BuildContext ctx) {
    final n = widget.points.length;
    if (n == 0) return;
    final dx = localPosition.dx.clamp(0.0, width);
    final index = ((dx / width) * (n - 1)).round().clamp(0, n - 1);
    setState(() {
      _tapX = dx;
      _selectedIndex = index;
      final p = widget.points[index];
      final rel = ((p.timestamp - widget.points.first.timestamp) / 1000).round();
      _overlayText = '${p.speedKmh.toStringAsFixed(1)} km/h';
      final minutes = rel ~/ 60;
      final seconds = rel % 60;
      _overlaySubtitle = '${minutes.toString().padLeft(2, '0')}:${seconds.toString().padLeft(2, '0')}';
    });

    // notify parent (if any) about the selected index so the map can sync
    widget.onIndexChanged?.call(index);

    // compute global position of the gesture relative to the screen
    final renderBox = ctx.findRenderObject() as RenderBox?;
    if (renderBox != null) {
      // compute chart top-left in global coords and place overlay at a fixed vertical offset above the chart
      final globalOrigin = renderBox.localToGlobal(Offset.zero);
      final globalX = (globalOrigin.dx + localPosition.dx).clamp(0.0, MediaQuery.of(ctx).size.width);
      final fixedTop = (globalOrigin.dy - 48.0).clamp(8.0, MediaQuery.of(ctx).size.height - 48.0);
      _overlayGlobal = Offset(globalX, fixedTop);
      _ensureOverlay();
    }
  }

  void _clearSelectionDelayed() {
    Future.delayed(const Duration(milliseconds: 600)).then((_) {
      if (mounted) {
        setState(() {
          _tapX = null;
          _selectedIndex = null;
        });
        _removeOverlay();
        widget.onIndexChanged?.call(null);
      }
    });
  }

  void _ensureOverlay() {
    if (_overlayEntry == null) {
      _overlayEntry = OverlayEntry(builder: (context) {
        if (_overlayGlobal == null) return const SizedBox.shrink();
        // position overlay at the fixed global top we computed, and offset horizontally
        final left = (_overlayGlobal!.dx - 40).clamp(8.0, MediaQuery.of(context).size.width - 88.0);
        final top = (_overlayGlobal!.dy).clamp(8.0, MediaQuery.of(context).size.height - 48.0);
        return Positioned(
          left: left,
          top: top,
          child: Material(
              elevation: 8,
              borderRadius: BorderRadius.circular(6),
              color: const Color(0xFF1E88E5),
              child: Padding(
                padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 6),
                child: Column(
                  mainAxisSize: MainAxisSize.min,
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Center(child: Text(_overlayText, style: Theme.of(context).textTheme.bodyMedium?.copyWith(fontWeight: FontWeight.bold, color: Colors.white))),
                    Text(_overlaySubtitle, style: Theme.of(context).textTheme.bodySmall?.copyWith(color: Colors.white), textAlign: TextAlign.left),
                  ],
                ),
              ),
            ),
        );
      });
      Overlay.of(context).insert(_overlayEntry!);
    } else {
      _overlayEntry!.markNeedsBuild();
    }
  }

  void _removeOverlay() {
    _overlayEntry?.remove();
    _overlayEntry = null;
  }

  @override
  Widget build(BuildContext context) {
    return LayoutBuilder(builder: (context, constraints) {
      final totalW = constraints.maxWidth;
      final h = constraints.maxHeight;
      const leftW = 52.0;
      const yLabelGap = 12.0; // horizontal gap between axis and labels
      final chartW = (totalW - leftW).clamp(40.0, totalW);

      // compute speeds and min/max (force ymin = 0)
      final speeds = widget.points.map((p) => p.speedKmh).toList();
      double minS = 0.0;
      double maxS = speeds.reduce((a, b) => a > b ? a : b);
      // mean speed
      final meanSpeed = speeds.isNotEmpty ? (speeds.reduce((a, b) => a + b) / speeds.length) : 0.0;
      if (maxS <= 0) maxS = 1.0;
      const int desiredSegments = 6; // number of segments (grid rows)

      // compute a "nice" step for ticks (1,2,5 * 10^exp)
      double rawStep = (maxS - minS) / desiredSegments;
      double exp = (rawStep > 0) ? math.pow(10, (math.log(rawStep) / math.ln10).floor()).toDouble() : 1.0;
      final candidates = [1.0, 2.0, 5.0, 10.0];
      double step = exp;
      for (final c in candidates) {
        if (exp * c >= rawStep) {
          step = exp * c;
          break;
        }
      }

      // create tick values starting at 0 up to a nice max
      final niceMax = (maxS / step).ceil() * step;
      final List<double> yTicks = [];
      for (double v = 0; v <= niceMax + 0.0001; v += step) {
        yTicks.add(double.parse(v.toStringAsFixed(6)));
      }

      // reserve space for x-axis labels below the chart plus a small buffer to avoid overflow
      const xAxisHeight = 30.0;
      const extraBuffer = 45.0;
      const topInset = 8.0; // small inset to avoid clipping of topmost y-label/title
      final chartH = (h - xAxisHeight - extraBuffer - topInset).clamp(40.0, h - extraBuffer - topInset);

      // compute x-axis tick seconds and fractions once (used for both painter and labels)
      final startMs = widget.points.first.timestamp;
      final endMs = widget.points.last.timestamp;
      final duration = (endMs - startMs).clamp(0, 1 << 60);
      final durationSec = (duration / 1000).round();

      List<int> tickSeconds;
      if (durationSec <= 0) {
        tickSeconds = [0];
      } else if (durationSec < 10) {
        tickSeconds = [for (int t = 0; t <= durationSec; t += 1) t];
      } else if (durationSec < 60) {
        tickSeconds = [for (int t = 0; t <= durationSec; t += 10) t];
      } else if (durationSec < 600) {
        // use 2-minute spacing for durations under 10 minutes
        tickSeconds = [for (int t = 0; t <= durationSec; t += 120) t];
      } else if (durationSec < 3600) {
        tickSeconds = [for (int t = 0; t <= durationSec; t += 300) t];
      } else {
        tickSeconds = [for (int t = 0; t <= durationSec; t += 3600) t];
      }

      List<int> normalizeTicks(List<int> ticks) {
        const int maxTicks = 6;
        if (ticks.length <= maxTicks) return ticks;
        int step = (ticks.length > 1) ? ticks[1] - ticks[0] : 1;
        while (ticks.length > maxTicks) {
          step = (step * 2).clamp(1, durationSec);
          ticks = [for (int t = 0; t <= durationSec; t += step) t];
          if (step >= durationSec) break;
        }
        if (!ticks.contains(durationSec)) ticks = [...ticks, durationSec];
        return ticks;
      }

      final ticks = normalizeTicks(tickSeconds);
      final tickFractions = ticks.map((s) => durationSec > 0 ? (s / durationSec) : 0.0).toList();

      String fmtFromSeconds(int totalSeconds) {
        final minutes = totalSeconds ~/ 60;
        final seconds = totalSeconds % 60;
        return '${minutes.toString().padLeft(2, '0')}:${seconds.toString().padLeft(2, '0')}';
      }

      return Padding(
        padding: const EdgeInsets.only(top: topInset),
        child: Row(crossAxisAlignment: CrossAxisAlignment.start, children: [
        SizedBox(
          width: leftW,
          height: (chartH + xAxisHeight + 1),
          child: Stack(
            children: [
              for (var i = 0; i < yTicks.length; i++) ...[
                Builder(builder: (ctx) {
                  final label = yTicks[i];
                  final textStyle = Theme.of(ctx).textTheme.bodySmall ?? const TextStyle(fontSize: 12);
                  final labelBoxWidth = (leftW - 6 - yLabelGap).clamp(8.0, leftW - 6);
                  final tp = TextPainter(
                    text: TextSpan(text: label.toStringAsFixed((label % 1 == 0) ? 0 : 1), style: textStyle),
                    textDirection: Directionality.of(ctx),
                  )..layout(minWidth: 0, maxWidth: labelBoxWidth);

                  final frac = (yTicks[i] - minS) / (niceMax - minS);
                  final rawTop = (1 - frac) * chartH - tp.height / 2;
                  final topPos = rawTop.clamp(0.0, chartH - tp.height);

                  return Positioned(
                    top: topPos,
                    right: yLabelGap,
                    child: SizedBox(
                      width: labelBoxWidth,
                      child: Text(
                        label.toStringAsFixed((label % 1 == 0) ? 0 : 1),
                        textAlign: TextAlign.right,
                        style: textStyle,
                      ),
                    ),
                  );
                }),
              ],
            ],
          ),
        ),
        Expanded(
            child: GestureDetector(
              onPanDown: (d) => _onDrag(d.localPosition, chartW, context),
              onPanUpdate: (d) => _onDrag(d.localPosition, chartW, context),
              onPanEnd: (e) => _clearSelectionDelayed(),
              onTapDown: (d) => _onDrag(d.localPosition, chartW, context),
              onTapUp: (d) => _clearSelectionDelayed(),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.stretch,
              children: [
                SizedBox(
                  height: chartH,
                  child: Stack(
                    children: [
                      CustomPaint(
                        size: Size(chartW, chartH),
                        painter: SpeedProfilePainter(speeds, highlightIndex: _selectedIndex, yTicks: yTicks, xTicks: tickFractions, meanS: meanSpeed, minS: minS, maxS: niceMax),
                      ),
                      if (_tapX != null && _selectedIndex != null) ...[
                        Positioned(
                          left: (_tapX! - 0.5).clamp(0.0, chartW - 1.0),
                          top: 0,
                          bottom: 0,
                          child: Container(width: 1, color: Theme.of(context).colorScheme.onSurface.withAlpha(31)),
                        ),
                      ],
                    ],
                  ),
                ),
                // x-axis tick labels placed below the plot (seconds relative to first point)
                SizedBox(height: 1),
                Padding(
                  padding: const EdgeInsets.only(top: 0, left: 4, right: 4),
                  child: Builder(builder: (ctx) {
                    return Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: [
                        SizedBox(
                          height: xAxisHeight,
                          width: chartW,
                          child: Stack(
                            children: ticks.asMap().entries.map((entry) {
                              final idx = entry.key;
                              final s = entry.value;
                              // hide the leading 0 label (gridline remains)
                              if (idx == 0 && s == 0) return const SizedBox.shrink();
                              final f = durationSec > 0 ? (s / durationSec) : 0.0;
                              final alignX = -1.0 + 2.0 * f; // maps 0..1 -> -1..1 for Align
                              return Align(
                                alignment: Alignment(alignX, 0),
                                child: Text(fmtFromSeconds(s), style: Theme.of(ctx).textTheme.bodySmall?.copyWith(color: Theme.of(ctx).colorScheme.onSurface)),
                              );
                            }).toList(),
                          ),
                        ),
                        // const SizedBox(height: 8),
                        Row(
                          children: [
                            Expanded(
                              child: Text('Max Speed', style: Theme.of(ctx).textTheme.bodyMedium?.copyWith(color: Theme.of(ctx).colorScheme.onSurface)),
                            ),
                            Text('${maxS.toStringAsFixed(1)} km/h', style: Theme.of(ctx).textTheme.bodyMedium?.copyWith(fontWeight: FontWeight.w600, color: Theme.of(ctx).colorScheme.onSurface)),
                          ],
                        ),
                        const SizedBox(height: 4),
                        Row(
                          children: [
                            Expanded(
                              child: Text('Mean Speed', style: Theme.of(ctx).textTheme.bodyMedium?.copyWith(color: Theme.of(ctx).colorScheme.onSurface)),
                            ),
                            Text('${meanSpeed.toStringAsFixed(1)} km/h', style: Theme.of(ctx).textTheme.bodyMedium?.copyWith(fontWeight: FontWeight.w600, color: Theme.of(ctx).colorScheme.onSurface)),
                          ],
                        ),
                      ],
                    );
                  }),
                ),
              ],
            ),
          ),
        ),
      ]),
    );
    });
  }
}
