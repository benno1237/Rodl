import 'dart:math';
import 'package:flutter/material.dart';
import '../models/settings.dart';

/// A modular border/animation wrapper for LED effects.
///
/// Provide an [effectIndex] and [baseColor]; the widget will animate
/// the border differently depending on the effect. The [child] should
/// be the tappable content (typically a Material/InkWell).
class EffectBorder extends StatefulWidget {
  // Canonical effect name (must match a key in `Settings.effects`). This
  // is the only external representation used by the UI; the provider is
  // responsible for translating names to integer IDs when communicating
  // with the backend.
  final String effectName;
  final bool selected;
  final Color baseColor;
  final int segments;
  final double height;
  final Widget child;

  const EffectBorder({
    super.key,
    required this.effectName,
    required this.selected,
    required this.baseColor,
    this.segments = 80,
    this.height = 52,
    required this.child,
  });
 

  @override
  State<EffectBorder> createState() => _EffectBorderState();
}

class _EffectBorderState extends State<EffectBorder> with SingleTickerProviderStateMixin {
  late final AnimationController _controller;
  late final Random _rand;
  late List<Color> _segmentColors;
  double _lastUpdateT = 0.0;
  int _racingPos = 0;

  @override
  void initState() {
    super.initState();

    // Use the canonical effect name and metadata from Settings.effects.
    final name = widget.effectName;
    final spec = Settings.effects[name];

    // Choose a sensible duration per-effect. Controller always runs but
    // some effects use it only when selected. Prefer a per-effect override
    // from the spec when available; otherwise use the `defaultSpeed` from
    // `Settings.effects` (interpreted as milliseconds).
    final duration = spec?.durationOverride ?? Duration(milliseconds: Settings.getDefaultSpeedForName(name));

    _controller = AnimationController(vsync: this, duration: duration)..repeat();
    _rand = Random();
    _segmentColors = List.generate(widget.segments, (_) => widget.baseColor);
  }

  @override
  @override
  void didUpdateWidget(covariant EffectBorder oldWidget) {
    super.didUpdateWidget(oldWidget);

    // If the effect changed, pick up the new per-effect duration and
    // update the controller so the animation speed reflects the
    // currently selected effect immediately.
    if (oldWidget.effectName != widget.effectName) {
      final name = widget.effectName;
      final spec = Settings.effects[name];
      final newDuration = spec?.durationOverride ?? Duration(milliseconds: Settings.getDefaultSpeedForName(name));
      if (_controller.duration != newDuration) {
        _controller.duration = newDuration;
        if (_controller.isAnimating) _controller.repeat();
      }

      // Reset effect-specific state so the new effect starts cleanly.
      _lastUpdateT = 0.0;
      _racingPos = 0;
      _segmentColors = List.generate(widget.segments, (_) => widget.baseColor);
      return;
    }

    // If only segments or base color changed, refresh the segment buffer.
    if (oldWidget.segments != widget.segments) {
      _segmentColors = List.generate(widget.segments, (_) => widget.baseColor);
    } else if (oldWidget.baseColor != widget.baseColor) {
      for (int i = 0; i < _segmentColors.length; i++) {
        _segmentColors[i] = widget.baseColor;
      }
    }
  }

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return AnimatedBuilder(
      animation: _controller,
      builder: (context, child) {
        final t = _controller.value;
        // Resolve mapping each build from the canonical name and metadata.
        final name = widget.effectName;
        final spec = Settings.effects[name];

        Color borderColor;
        double borderWidth;
        // Lower opacity for non-selected effects but still show the animation.
        const double deselectedOpacity = 0.28;

        switch (name) {
          case 'breathing':
            final fullAlpha = (t < 0.5) ? 0.35 + 1.2 * t : 0.35 + 1.2 * (1.0 - t); // pulsating when selected
            final lowAlpha = 0.15 + 0.35 * t; // subdued pulse when not selected
            borderColor = widget.baseColor.withAlpha(((widget.selected ? fullAlpha : lowAlpha) * 255).round());
            borderWidth = widget.selected ? 2.5 : 1.0;
            break;
          case 'rainbow':
            final hue = (t * 360.0) % 360.0;
            final rainbow = HSVColor.fromAHSV(1.0, hue, 0.85, 0.9).toColor();
            borderColor = rainbow.withAlpha(((widget.selected ? 0.9 : deselectedOpacity) * 255).round());
            borderWidth = widget.selected ? 2.5 : 1.0;
            break;
          case 'police':
            final policeColor = (t < 0.5) ? Colors.blue : Colors.red;
            borderColor = policeColor.withAlpha(((widget.selected ? 1.0 : deselectedOpacity) * 255).round());
            borderWidth = widget.selected ? 2.5 : 1.0;
            break;
          case 'strobe':
            final flash = t > 0.5;
            if (widget.selected) {
              borderColor = widget.baseColor.withAlpha(flash ? 255 : 0);
              borderWidth = flash ? 3.0 : 0.8;
            } else {
              borderColor = widget.baseColor.withAlpha(((flash ? deselectedOpacity : deselectedOpacity * 0.25) * 255).round());
              borderWidth = 1.0;
            }
            break;
          case 'random':
            if ((t - _lastUpdateT).abs() > 0.18) {
              _lastUpdateT = t;
              for (int i = 0; i < widget.segments; i++) {
                if (_rand.nextDouble() < 0.25) {
                  _segmentColors[i] = HSVColor.fromAHSV(
                    1.0,
                    _rand.nextDouble() * 360.0,
                    0.8 + _rand.nextDouble() * 0.2,
                    0.7 + _rand.nextDouble() * 0.3,
                  ).toColor();
                } else {
                  _segmentColors[i] = widget.baseColor.withAlpha((deselectedOpacity * 255).round());
                }
              }
            }
            borderColor = widget.baseColor.withAlpha((deselectedOpacity * 255).round());
            borderWidth = widget.selected ? 2.0 : 1.0;
            break;
          case 'rainbow_wipe':
            final flowHueShift = (t * 360.0) % 360.0;
            for (int i = 0; i < widget.segments; i++) {
              final hue = ((i / widget.segments) * 360.0 + flowHueShift) % 360.0;
              final color = HSVColor.fromAHSV(1.0, hue, 0.85, 0.9).toColor();
              final alpha = widget.selected ? 0.9 : deselectedOpacity;
              _segmentColors[i] = color.withAlpha((alpha * 255).round());
            }
            borderColor = widget.baseColor.withAlpha((deselectedOpacity * 255).round());
            borderWidth = widget.selected ? 2.0 : 1.0;
            break;
          case 'static_wipe':
            if (t < 0.5) {
              final phaseT = (t / 0.5).clamp(0.0, 1.0);
              final fillCount = (phaseT * widget.segments).floor();
              for (int i = 0; i < widget.segments; i++) {
                if (i <= fillCount) {
                  _segmentColors[i] = widget.baseColor.withAlpha(255);
                } else {
                  _segmentColors[i] = widget.baseColor.withAlpha((deselectedOpacity * 255).round());
                }
              }
            } else {
              final phaseT = ((t - 0.5) / 0.5).clamp(0.0, 1.0);
              final clearCount = (phaseT * widget.segments).floor();
              for (int i = 0; i < widget.segments; i++) {
                if (i <= clearCount) {
                  _segmentColors[i] = widget.baseColor.withAlpha(0);
                } else {
                  _segmentColors[i] = widget.baseColor.withAlpha(255);
                }
              }
            }
            borderColor = widget.baseColor.withAlpha((deselectedOpacity * 255).round());
            borderWidth = widget.selected ? 2.0 : 1.0;
            break;
          case 'racing':
            final active = (t * widget.segments).floor() % widget.segments;
            if (active != _racingPos) {
              _racingPos = active;
              for (int i = 0; i < widget.segments; i++) {
                _segmentColors[i] = widget.baseColor.withAlpha((deselectedOpacity * 255).round());
              }
              _segmentColors[_racingPos] = widget.baseColor.withAlpha(255);
            }
            borderColor = widget.baseColor.withAlpha((deselectedOpacity * 255).round());
            borderWidth = widget.selected ? 2.0 : 1.0;
            break;
          default:
            borderColor = widget.baseColor.withAlpha(((widget.selected ? 0.8 : deselectedOpacity) * 255).round());
            borderWidth = widget.selected ? 2.0 : 1.0;
        }

        // Reserve a fixed total inset (border + inner padding) so the child
        // content doesn't shift when the border thickness changes (strobe).
        const double contentInset = 4.0; // desired total inset (borderWidth + inner padding)
        // If the effect is segmented, render via the custom painter.
        final isSegmented = spec?.segmented ?? false;
        if (isSegmented) {
          final bgColor = widget.selected ? widget.baseColor.withAlpha((0.06 * 255).round()) : null;
          return Container(
            height: widget.height,
            decoration: BoxDecoration(
              color: bgColor,
              borderRadius: BorderRadius.circular(12),
            ),
            child: CustomPaint(
              painter: _SegmentBorderPainter(
                segmentColors: List<Color>.from(_segmentColors),
                radius: 12,
                thickness: borderWidth,
              ),
              child: Padding(
                padding: EdgeInsets.all((contentInset - borderWidth).clamp(0.0, contentInset)),
                child: child,
              ),
            ),
          );
        }

        return Container(
          height: widget.height,
          decoration: BoxDecoration(
            color: widget.selected ? widget.baseColor.withAlpha((0.06 * 255).round()) : null,
            borderRadius: BorderRadius.circular(12),
            border: Border.all(color: borderColor, width: borderWidth),
          ),
          child: Padding(
            padding: EdgeInsets.all((contentInset - borderWidth).clamp(0.0, contentInset)),
            child: child,
          ),
        );
      },
      child: widget.child,
    );
  }
}

class _SegmentBorderPainter extends CustomPainter {
  final List<Color> segmentColors;
  final double radius;
  final double thickness;

  _SegmentBorderPainter({required this.segmentColors, this.radius = 12, this.thickness = 6});

  @override
  void paint(Canvas canvas, Size size) {
    if (segmentColors.isEmpty) return;

    final w = size.width;
    final h = size.height;

    // Inset the path so the stroke sits fully inside the bounds.
    final inset = thickness / 2.0 + 1.0; // small extra inset
    final rect = Rect.fromLTWH(inset, inset, (w - inset * 2).clamp(0.0, w), (h - inset * 2).clamp(0.0, h));
    final r = Radius.circular(max(0.0, radius - inset));
    final rrect = RRect.fromRectAndRadius(rect, r);

    final path = Path()..addRRect(rrect);
    final metrics = path.computeMetrics().toList();
    if (metrics.isEmpty) return;
    final totalLen = metrics.map((m) => m.length).reduce((a, b) => a + b);
    final segCount = segmentColors.length;
    final segLen = totalLen / segCount;

    final paint = Paint()
      ..style = PaintingStyle.stroke
      ..strokeWidth = thickness
      ..strokeCap = StrokeCap.butt;

    // draw each segment as an extracted subpath stroke with rounded caps
    double offset = 0.0;
    final gapFactor = 0; // keep a small gap between segments
    final drawLen = segLen * (1.0 - gapFactor);

    for (int i = 0; i < segCount; i++) {
      final start = offset;

      // extract across metrics (handles single metric normally)
      double remaining = drawLen;
      double cursor = start;
      Path segmentPath = Path();
      for (final m in metrics) {
        if (cursor >= m.length) {
          cursor -= m.length;
          continue;
        }
        final available = m.length - cursor;
        final take = min(available, remaining);
        segmentPath.addPath(m.extractPath(cursor, cursor + take), Offset.zero);
        remaining -= take;
        cursor = 0.0;
        if (remaining <= 0) break;
      }

      paint.color = segmentColors[i];
      canvas.drawPath(segmentPath, paint);

      offset += segLen;
    }
  }

  @override
  bool shouldRepaint(covariant _SegmentBorderPainter oldDelegate) {
    if (oldDelegate.radius != radius || oldDelegate.thickness != thickness) return true;
    final a = oldDelegate.segmentColors;
    final b = segmentColors;
    if (a.length != b.length) return true;
    for (int i = 0; i < a.length; i++) {
      if (a[i].toARGB32() != b[i].toARGB32()) return true;
    }
    return false;
  }
}
