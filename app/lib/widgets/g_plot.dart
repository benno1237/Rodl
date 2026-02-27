import 'package:flutter/material.dart';
import 'dart:math';

class GPlot extends StatelessWidget {
  final double gx;
  final double gy;
  final double maxG;

  const GPlot({
    super.key,
    required this.gx,
    required this.gy,
    this.maxG = 2.0,
  });

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: Colors.grey.shade900,
        borderRadius: BorderRadius.circular(20),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withOpacity(0.4),
            blurRadius: 10,
            offset: const Offset(0, 6),
          )
        ],
      ),
      child: AspectRatio(
        aspectRatio: 1,
        child: CustomPaint(
          painter: _GPlotPainter(gx, gy, maxG),
        ),
      ),
    );
  }
}

class _GPlotPainter extends CustomPainter {
  final double gx;
  final double gy;
  final double maxG;

  _GPlotPainter(this.gx, this.gy, this.maxG);

  @override
  void paint(Canvas canvas, Size size) {
    final center = Offset(size.width / 2, size.height / 2);
    final radius = size.width / 2;

    final gridPaint = Paint()
      ..color = Colors.grey.shade700
      ..style = PaintingStyle.stroke
      ..strokeWidth = 1;

    final axisPaint = Paint()
      ..color = Colors.grey.shade500
      ..strokeWidth = 1.5;

    // Draw concentric circles
    for (double g = 0.5; g <= maxG; g += 0.5) {
      final r = radius * (g / maxG);
      canvas.drawCircle(center, r, gridPaint);

      // Draw circle labels (top of circle)
      final textPainter = TextPainter(
        text: TextSpan(
          text: "${g.toStringAsFixed(1)}g",
          style: const TextStyle(
            color: Colors.white70,
            fontSize: 12,
          ),
        ),
        textDirection: TextDirection.ltr,
      );

      textPainter.layout();
      textPainter.paint(
        canvas,
        Offset(center.dx - textPainter.width / 2, center.dy - r - 14),
      );
    }

    // Draw axes
    canvas.drawLine(
      Offset(center.dx, 0),
      Offset(center.dx, size.height),
      axisPaint,
    );

    canvas.drawLine(
      Offset(0, center.dy),
      Offset(size.width, center.dy),
      axisPaint,
    );

    // Axis labels
    _drawAxisLabel(canvas, "Longitudinal +G", Offset(center.dx + 8, 4));
    _drawAxisLabel(canvas, "Longitudinal -G",
        Offset(center.dx + 8, size.height - 20));

    _drawAxisLabel(canvas, "Lateral -G", Offset(4, center.dy + 4));
    _drawAxisLabel(canvas, "Lateral +G",
        Offset(size.width - 80, center.dy + 4));

    // Draw acceleration dot
    final dotPaint = Paint()
      ..color = Colors.redAccent
      ..style = PaintingStyle.fill;

    final scaledX = (gx / maxG) * radius;
    final scaledY = (gy / maxG) * radius;

    final dotPosition = Offset(
      center.dx + scaledX,
      center.dy - scaledY,
    );

    canvas.drawCircle(dotPosition, 6, dotPaint);
  }

  void _drawAxisLabel(Canvas canvas, String text, Offset position) {
    final textPainter = TextPainter(
      text: TextSpan(
        text: text,
        style: const TextStyle(
          color: Colors.white54,
          fontSize: 11,
        ),
      ),
      textDirection: TextDirection.ltr,
    );

    textPainter.layout();
    textPainter.paint(canvas, position);
  }

  @override
  bool shouldRepaint(covariant _GPlotPainter oldDelegate) {
    return oldDelegate.gx != gx || oldDelegate.gy != gy;
  }
}