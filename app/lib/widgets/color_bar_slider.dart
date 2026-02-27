import 'package:flutter/material.dart';

class ColorBarSlider extends StatelessWidget {
  final double value;
  final double min;
  final double max;
  final Gradient gradient;
  final ValueChanged<double> onChanged;
  final double cornerRadius = 10;
  final double height = 32;

  const ColorBarSlider({
    super.key,
    required this.value,
    required this.min,
    required this.max,
    required this.gradient,
    required this.onChanged,
  });

  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      onHorizontalDragUpdate: (details) {
        _handleDrag(details.localPosition.dx, context);
      },
      onTapDown: (details) {
        _handleDrag(details.localPosition.dx, context);
      },
      child: Container(
        height: height,
        decoration: BoxDecoration(
          border: Border.all(color: Colors.black, width: 2),
          borderRadius: BorderRadius.circular(cornerRadius),
        ),
        child: ClipRRect(
          borderRadius: BorderRadius.circular(cornerRadius - 2), // inner rounding
          child: LayoutBuilder(
            builder: (context, constraints) {
              final width = constraints.maxWidth;
              final percentage = (value - min) / (max - min);
              final fillWidth = (width * percentage).clamp(0.0, width);

              return Stack(
                children: [
                  // Background (optional, but nice for contrast)
                  Container(color: Colors.transparent),

                  // Gradient fill (no gap)
                  AnimatedContainer(
                    duration: const Duration(milliseconds: 120),
                    width: fillWidth,
                    decoration: BoxDecoration(
                      gradient: gradient,
                    ),
                  ),
                ],
              );
            },
          ),
        ),
      ),
    );
  }

  void _handleDrag(double localX, BuildContext context) {
    final box = context.findRenderObject() as RenderBox?;
    if (box == null) return;

    final width = box.size.width;
    final clamped = (localX / width).clamp(0.0, 1.0);
    final newValue = min + (max - min) * clamped;

    onChanged(newValue);
  }
}