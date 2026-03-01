import 'package:flutter/material.dart';

/// A horizontal color bar slider that supports ranges crossing zero.
///
/// When `min < 0 && max > 0` the zero position is treated as the origin
/// and fills grow left (negative) or right (positive) from that center.
class SplitColorBarSlider extends StatelessWidget {
  final double value;
  final double min;
  final double max;
  final Gradient gradient;
  final ValueChanged<double> onChanged;
  final double cornerRadius;
  final double height;
  final double borderWidth;
  final bool showZeroMarker;

  const SplitColorBarSlider({
    super.key,
    required this.value,
    required this.min,
    required this.max,
    required this.gradient,
    required this.onChanged,
    this.cornerRadius = 10.0,
    this.height = 32.0,
    this.borderWidth = 2.0,
    this.showZeroMarker = true,
  });

  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      onHorizontalDragUpdate: (details) => _handleDrag(details.localPosition.dx, context),
      onTapDown: (details) => _handleDrag(details.localPosition.dx, context),
      child: Container(
        height: height,
        decoration: BoxDecoration(
          border: Border.all(color: Colors.black, width: borderWidth),
          borderRadius: BorderRadius.circular(cornerRadius),
        ),
        child: ClipRRect(
          borderRadius: BorderRadius.circular(cornerRadius - borderWidth),
          child: LayoutBuilder(builder: (context, constraints) {
            final width = constraints.maxWidth;
            final range = (max - min);
            final percentage = ((value - min) / (range)).clamp(0.0, 1.0);

            final crossesZero = min < 0 && max > 0;

            // When range crosses zero we render two independent areas:
            // - left area (from start to zero) where negative fills grow to the right
            // - right area (from zero to end) where positive fills grow to the right
            // This keeps the zero marker fixed and avoids abrupt position jumps.
            if (crossesZero) {
              final zeroPct = ((0 - min) / range).clamp(0.0, 1.0);
              final zeroPos = width * zeroPct;

              // Negative (left) fill width and positive (right) fill width in absolute pixels
              final negFill = (percentage < zeroPct) ? width * (zeroPct - percentage) : 0.0;
              final posFill = (percentage > zeroPct) ? width * (percentage - zeroPct) : 0.0;

              return Stack(children: [
                // Background
                Container(color: Colors.transparent),

                // Left area (fixed width) with negative fill aligned to the right (touches zero)
                Positioned(
                  left: 0,
                  top: 0,
                  bottom: 0,
                  child: SizedBox(
                    width: zeroPos,
                    height: height,
                    child: Stack(
                      children: [
                        Container(color: Colors.transparent),
                        if (negFill > 0)
                          Align(
                            alignment: Alignment.centerRight,
                            child: AnimatedContainer(
                              duration: const Duration(milliseconds: 120),
                              width: negFill.clamp(0.0, zeroPos),
                              height: height,
                              decoration: BoxDecoration(gradient: gradient),
                            ),
                          ),
                      ],
                    ),
                  ),
                ),

                // Right area (fixed width) with positive fill aligned to the left (touches zero)
                Positioned(
                  left: zeroPos,
                  top: 0,
                  bottom: 0,
                  child: SizedBox(
                    width: (width - zeroPos),
                    height: height,
                    child: Stack(
                      children: [
                        Container(color: Colors.transparent),
                        if (posFill > 0)
                          Align(
                            alignment: Alignment.centerLeft,
                            child: AnimatedContainer(
                              duration: const Duration(milliseconds: 120),
                              width: posFill.clamp(0.0, width - zeroPos),
                              height: height,
                              decoration: BoxDecoration(gradient: gradient),
                            ),
                          ),
                      ],
                    ),
                  ),
                ),

                // Optional zero marker when range crosses zero
                if (showZeroMarker)
                  Positioned(
                    left: zeroPos,
                    top: 0,
                    bottom: 0,
                    child: Container(
                      width: 2,
                      color: Theme.of(context).colorScheme.onSurface.withAlpha((0.6 * 255).round()),
                    ),
                  ),
              ]);
            }

            // Non-split case (no zero crossing) - simple left-to-right fill
            final fillWidth = width * percentage;
            return Stack(children: [
              Container(color: Colors.transparent),
              if (fillWidth > 0)
                Positioned(
                  left: 0,
                  top: 0,
                  bottom: 0,
                  child: AnimatedContainer(
                    duration: const Duration(milliseconds: 120),
                    width: fillWidth.clamp(0.0, width),
                    decoration: BoxDecoration(gradient: gradient),
                  ),
                ),
            ]);
          }),
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
