import 'dart:math';
import 'package:flutter/material.dart';

class ColorWheel extends StatefulWidget {
  final Color initialColor;
  final ValueChanged<Color> onChanged;
  final double size;

  const ColorWheel({super.key, required this.initialColor, required this.onChanged, this.size = 260});

  @override
  State<ColorWheel> createState() => _ColorWheelState();
}

class _ColorWheelState extends State<ColorWheel> {
  late double hue;
  late double saturation;
  late double value;
  

  @override
  void initState() {
    super.initState();
    final h = HSVColor.fromColor(widget.initialColor);
    hue = h.hue;
    saturation = h.saturation;
    value = h.value;
  }

  @override
  void didUpdateWidget(covariant ColorWheel oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (oldWidget.size != widget.size) {
      setState(() {});
      return;
    }

    // Only update saturation/value from an external initialColor change.
    // Do not overwrite `hue` here so that the hue ring and the SV area remain decoupled
    // during interactive updates coming from this widget's onChanged callback.
    if (oldWidget.initialColor != widget.initialColor) {
      final h = HSVColor.fromColor(widget.initialColor);
      saturation = h.saturation;
      value = h.value;
      setState(() {});
    }
  }

  void _updateFromOffset(Offset localOffset) {
    final center = Offset(widget.size / 2, widget.size / 2);
    final dx = localOffset.dx - center.dx;
    final dy = localOffset.dy - center.dy;
    double angle = atan2(dy, dx);
    double deg = (angle * 180 / pi);
    final newHue = (deg + 360) % 360;

    setState(() {
      hue = newHue;
    });

    final combined = HSVColor.fromAHSV(1.0, hue, saturation, value);
    widget.onChanged(combined.toColor());
  }

  

  void _updateSatValFromLocal(Offset localOffset, double width, double height) {
    final sx = (localOffset.dx / width).clamp(0.0, 1.0);
    final sy = (localOffset.dy / height).clamp(0.0, 1.0);
    final sat = sx;
    final val = 1.0 - sy;
    setState(() {
      saturation = sat;
      value = val;
    });

    final combined = HSVColor.fromAHSV(1.0, hue, saturation, value);
    widget.onChanged(combined.toColor());
  }

  @override
  Widget build(BuildContext context) {
    final size = widget.size;
    final innerRatio = 0.8;
    final childSize = 20.0;

    return Column(
      mainAxisSize: MainAxisSize.min,
      children: [
        GestureDetector(
          onPanDown: (e) => _updateFromOffset(e.localPosition),
          onPanUpdate: (e) => _updateFromOffset(e.localPosition),
          onTapDown: (e) => _updateFromOffset(e.localPosition),
          child: SizedBox(
            width: size,
            height: size,
            child: LayoutBuilder(builder: (context, constraints) {
              final w = constraints.maxWidth;
              final h = constraints.maxHeight;
              final usedSize = w < h ? w : h;
              final outerR = usedSize / 2;
              final innerR = (usedSize * innerRatio) / 2;
              final ringR = (outerR + innerR) / 2;
              final center = Offset(w / 2, h / 2);
              final angleLocal = hue * pi / 180.0;
              final combinedColor = HSVColor.fromAHSV(1.0, hue, saturation, value);
              final childSizeLocal = childSize;
              final childR = childSizeLocal / 2;
              final pointerCenter = center + Offset(cos(angleLocal) * ringR, sin(angleLocal) * ringR);

              return Stack(
                children: [
              // Hue ring
              Positioned.fill(
                child: Center(
                  child: Container(
                    width: usedSize,
                    height: usedSize,
                    decoration: const BoxDecoration(
                      shape: BoxShape.circle,
                      gradient: SweepGradient(
                        colors: [
                          Colors.red,
                          Colors.yellow,
                          Colors.green,
                          Colors.cyan,
                          Colors.blue,
                          Color(0xFFFF00FF),
                          Colors.red,
                        ],
                      ),
                    ),
                  ),
                ),
              ),

              // Inner circle to create the ring effect and show the live preview
              Positioned(
                left: center.dx - innerR,
                top: center.dy - innerR,
                width: innerR * 2,
                height: innerR * 2,
                child: Container(
                  decoration: BoxDecoration(
                    color: Theme.of(context).dialogTheme.backgroundColor ?? Theme.of(context).colorScheme.surface,
                    shape: BoxShape.circle,
                  ),
                  child: Center(
                    child: Column(
                      mainAxisSize: MainAxisSize.min,
                      crossAxisAlignment: CrossAxisAlignment.center,
                      children: [
                        Container(
                          width: usedSize * 0.22,
                          height: usedSize * 0.22,
                          decoration: BoxDecoration(
                            color: combinedColor.toColor(),
                            shape: BoxShape.circle,
                            border: Border.all(color: Colors.white, width: 2),
                            boxShadow: [BoxShadow(color: Colors.black26, blurRadius: 4, offset: const Offset(0, 2))],
                          ),
                        ),
                        const SizedBox(height: 20),
                        Column(
                          mainAxisSize: MainAxisSize.min,
                          crossAxisAlignment: CrossAxisAlignment.start,
                          children: [
                            Text(
                              'HEX',
                              style: Theme.of(context).textTheme.bodySmall?.copyWith(color: Colors.grey[600]),
                            ),
                            Text(
                              '#${combinedColor.toColor().toARGB32().toRadixString(16).padLeft(8, '0').toUpperCase()}',
                              style: Theme.of(context).textTheme.bodyLarge?.copyWith(fontWeight: FontWeight.w600),
                            ),
                            Container(
                              width: usedSize * 0.4,
                              height: 1,
                              color: Colors.grey[600],
                              margin: const EdgeInsets.symmetric(vertical: 0),
                            ),
                            const SizedBox(height: 25),
                          ],
                        ),
                      ],
                    ),
                  ),
                ),
              ),

              // Subtle outer rim
              Positioned.fill(
                child: Container(
                  decoration: BoxDecoration(
                    shape: BoxShape.circle,
                    gradient: RadialGradient(
                      colors: [Colors.transparent, Colors.black.withAlpha((0.06 * 255).round())],
                      stops: const [0.9, 1.0],
                    ),
                  ),
                ),
              ),

                  // Pointer positioned precisely with left/top
                  Positioned(
                    left: pointerCenter.dx - childR,
                    top: pointerCenter.dy - childR,
                    width: childSizeLocal,
                    height: childSizeLocal,
                    child: Container(
                      decoration: BoxDecoration(
                        shape: BoxShape.circle,
                        border: Border.all(color: Colors.white, width: 2),
                        color: combinedColor.toColor(),
                        boxShadow: [BoxShadow(color: Colors.black, blurRadius: 4)],
                      ),
                    ),
                  ),
                ],
              );
            }),
          ),
        ),

            const SizedBox(height: 12),

            // Saturation/Value square (stretched to wheel width)
            LayoutBuilder(builder: (context, constraints) {
              final svWidth = constraints.maxWidth;
              final svHeight = (size * 0.54).clamp(120.0, size * 0.7);
              final selectorSize = 16.0;
              final sx = saturation * svWidth;
              final sy = (1.0 - value) * svHeight;

              return GestureDetector(
                onPanDown: (e) => _updateSatValFromLocal(e.localPosition, svWidth, svHeight),
                onPanUpdate: (e) => _updateSatValFromLocal(e.localPosition, svWidth, svHeight),
                onTapDown: (e) => _updateSatValFromLocal(e.localPosition, svWidth, svHeight),
                child: Center(
                  child: SizedBox(
                    width: svWidth,
                    height: svHeight,
                    child: Stack(
                      children: [
                        // base hue color (full sat/value)
                        Positioned.fill(
                          child: Container(
                            decoration: BoxDecoration(
                              color: HSVColor.fromAHSV(1.0, hue, 1.0, 1.0).toColor(),
                              borderRadius: BorderRadius.circular(8),
                            ),
                          ),
                        ),
                        // white -> transparent (left to right) for saturation
                        Positioned.fill(
                          child: Container(
                            decoration: BoxDecoration(
                              gradient: const LinearGradient(
                                begin: Alignment.centerLeft,
                                end: Alignment.centerRight,
                                colors: [Colors.white, Colors.transparent],
                              ),
                              borderRadius: BorderRadius.circular(8),
                            ),
                          ),
                        ),
                        // transparent -> black (top to bottom) for value
                        Positioned.fill(
                          child: Container(
                            decoration: BoxDecoration(
                              gradient: const LinearGradient(
                                begin: Alignment.topCenter,
                                end: Alignment.bottomCenter,
                                colors: [Colors.transparent, Colors.black],
                              ),
                              borderRadius: BorderRadius.circular(8),
                            ),
                          ),
                        ),

                        // selector
                        Positioned(
                          left: (sx - selectorSize / 2).clamp(0.0, svWidth - selectorSize),
                          top: (sy - selectorSize / 2).clamp(0.0, svHeight - selectorSize),
                          child: Container(
                            width: selectorSize,
                            height: selectorSize,
                            decoration: BoxDecoration(
                              shape: BoxShape.circle,
                              color: HSVColor.fromAHSV(1.0, hue, saturation, value).toColor(),
                              border: Border.all(color: Colors.white, width: 2),
                              boxShadow: [BoxShadow(color: Colors.black26, blurRadius: 4, offset: const Offset(0, 2))],
                            ),
                          ),
                        ),
                      ],
                    ),
                  ),
                ),
              );
            }),

            const SizedBox(height: 8),
      ],
    );
  }
}
