import 'package:flutter/material.dart';
import 'package:flutter/foundation.dart';
// import 'dart:ui' show Offset;

class GPlot extends StatefulWidget {
  final double gx;
  final double gy;
  final double maxG;
  final ValueListenable<Offset>? valueListenable;

  const GPlot({
    super.key,
    required this.gx,
    required this.gy,
    this.maxG = 2.0,
    this.valueListenable,
  });

  @override
  State<GPlot> createState() => _GPlotState();
}

class _GPlotState extends State<GPlot> {
  late final ValueListenable<Offset> _notifier;
  late final _GPlotPainter _painter;
  late final bool _ownsNotifier;

  @override
  void initState() {
    super.initState();
    if (widget.valueListenable != null) {
      _notifier = widget.valueListenable!;
      _ownsNotifier = false;
    } else {
      _notifier = ValueNotifier(Offset(widget.gx, widget.gy));
      _ownsNotifier = true;
    }
    _painter = _GPlotPainter(_notifier, widget.maxG);
  }

  @override
  void didUpdateWidget(covariant GPlot oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (_ownsNotifier && (oldWidget.gx != widget.gx || oldWidget.gy != widget.gy)) {
      // update internal notifier only when we own it
      ( _notifier as ValueNotifier<Offset>).value = Offset(widget.gx, widget.gy);
    }
    // If `maxG` changes rarely, we currently ignore recreating the painter.
  }

  @override
  void dispose() {
    if (_ownsNotifier) {
      (_notifier as ValueNotifier<Offset>).dispose();
    }
    _painter.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return RepaintBoundary(
      child: Container(
        padding: const EdgeInsets.all(16),
        decoration: BoxDecoration(
          color: Colors.grey.shade900,
          borderRadius: BorderRadius.circular(20),
          boxShadow: [
            BoxShadow(
              color: Colors.black.withAlpha((0.4 * 255).round()),
              blurRadius: 10,
              offset: const Offset(0, 6),
            )
          ],
        ),
        child: AspectRatio(
          aspectRatio: 1,
          child: CustomPaint(
            painter: _painter,
          ),
        ),
      ),
    );
  }
}

class _GPlotPainter extends CustomPainter {
  final ValueListenable<Offset> notifier;
  final double maxG;

  // Cached text painters
  late final List<TextPainter> _circleLabelPainters;
  late final List<double> _circleGs;
  late final TextPainter _axisTop;
  late final TextPainter _axisBottom;
  late final TextPainter _axisLeft;
  late final TextPainter _axisRight;

  _GPlotPainter(this.notifier, this.maxG) : super(repaint: notifier) {
    _circleGs = [];
    _circleLabelPainters = [];
    for (double g = 0.5; g <= maxG; g += 0.5) {
      _circleGs.add(g);
      final tp = TextPainter(
        text: TextSpan(
          text: "${g.toStringAsFixed(1)}g",
          style: const TextStyle(color: Colors.white70, fontSize: 12),
        ),
        textDirection: TextDirection.ltr,
      );
      tp.layout();
      _circleLabelPainters.add(tp);
    }

    _axisTop = TextPainter(
      text: const TextSpan(
        text: 'Longitudinal +G',
        style: TextStyle(color: Colors.white54, fontSize: 11),
      ),
      textDirection: TextDirection.ltr,
    )..layout();

    _axisBottom = TextPainter(
      text: const TextSpan(
        text: 'Longitudinal -G',
        style: TextStyle(color: Colors.white54, fontSize: 11),
      ),
      textDirection: TextDirection.ltr,
    )..layout();

    _axisLeft = TextPainter(
      text: const TextSpan(
        text: 'Lateral -G',
        style: TextStyle(color: Colors.white54, fontSize: 11),
      ),
      textDirection: TextDirection.ltr,
    )..layout();

    _axisRight = TextPainter(
      text: const TextSpan(
        text: 'Lateral +G',
        style: TextStyle(color: Colors.white54, fontSize: 11),
      ),
      textDirection: TextDirection.ltr,
    )..layout();
  }

  @override
  void paint(Canvas canvas, Size size) {
    final center = Offset(size.width / 2, size.height / 2);
    final radius = size.width / 2;

    final gridPaint = Paint()
      ..color = Colors.grey.shade700
      ..style = PaintingStyle.stroke
      ..strokeWidth = 1;

    final axisPaint = Paint()..color = Colors.grey.shade500..strokeWidth = 1.5;

    // Draw concentric circles
    for (int i = 0; i < _circleGs.length; i++) {
      final g = _circleGs[i];
      final r = radius * (g / maxG);
      canvas.drawCircle(center, r, gridPaint);

      final textPainter = _circleLabelPainters[i];
      textPainter.paint(canvas, Offset(center.dx - textPainter.width / 2, center.dy - r - 14));
    }

    // Draw axes
    canvas.drawLine(Offset(center.dx, 0), Offset(center.dx, size.height), axisPaint);
    canvas.drawLine(Offset(0, center.dy), Offset(size.width, center.dy), axisPaint);

    // Axis labels
    _axisTop.paint(canvas, Offset(center.dx + 8, 4));
    _axisBottom.paint(canvas, Offset(center.dx + 8, size.height - 20));
    _axisLeft.paint(canvas, Offset(4, center.dy + 4));
    _axisRight.paint(canvas, Offset(size.width - 80, center.dy + 4));

    // Draw acceleration dot
    final dotPaint = Paint()..color = Colors.redAccent..style = PaintingStyle.fill;

    final offset = notifier.value;
    final gx = offset.dx;
    final gy = offset.dy;

    final scaledX = (gx / maxG) * radius;
    final scaledY = (gy / maxG) * radius;

    final dotPosition = Offset(center.dx + scaledX, center.dy - scaledY);
    canvas.drawCircle(dotPosition, 6, dotPaint);
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => false;

  void dispose() {}
}