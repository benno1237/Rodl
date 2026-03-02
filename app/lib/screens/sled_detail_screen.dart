import 'dart:async';
import 'package:flutter/material.dart';
import 'package:flutter/foundation.dart';
import '../models/sled.dart';
import '../widgets/g_plot.dart';
import '../widgets/color_bar_slider.dart';
import '../widgets/split_color_bar_slider.dart';
import '../widgets/color_wheel.dart';
import '../widgets/effect_border.dart';
import 'package:provider/provider.dart';
import 'package:geolocator/geolocator.dart';
import '../providers/settings_provider.dart';
import '../models/settings.dart';
import '../models/gps_point.dart';
import '../providers/rides_provider.dart';
import '../models/ride.dart';
import '../services/foreground_location_service.dart' as fg_service;

class SledDetailScreen extends StatefulWidget {
  final Sled sled;

  const SledDetailScreen({super.key, required this.sled});

  @override
  State<SledDetailScreen> createState() => _SledDetailScreenState();
}

class _SledDetailScreenState extends State<SledDetailScreen> with SingleTickerProviderStateMixin {
  double gx = 0.0;
  double gy = 0.0;

  late final ValueNotifier<Offset> _gNotifier;
  StreamSubscription? _gSub;
  // Glow controller removed — per-effect animations live in `EffectBorder`.

  @override
  void setState(VoidCallback fn) {
    if (!mounted) return;
    super.setState(fn);
  }

  int _selectedIndex = 0;

  // Frontlight
  double frontlightValue = 0; // -100 to +100

  // GPS
  late final TextEditingController _gpsController;
  StreamSubscription<Position>? _positionSub;
  // Recording
  bool _isRecording = false;
  final List<GPSPoint> _recordedPoints = [];
  StreamSubscription<Position>? _recordingSub;

  // RGB strip
  double stripBrightness = 50;
  // Selected effect is stored in settings (int). UI will read from provider.
  Color selectedColor = Colors.blue;

  void _onItemTapped(int index) {
    setState(() {
      _selectedIndex = index;
    });
  }

  // removed _buildSplitColorBar helper — building split cbar inline for full control

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addPostFrameCallback((_) {
      context.read<SettingsProvider>().connect();
    });

    // Per-effect animations handled by `EffectBorder` widget.

    // Simulated live updates (replace later with BLE stream)
    _gNotifier = ValueNotifier(const Offset(0, 0));
    _gSub = Stream.periodic(const Duration(milliseconds: 100), (i) {
      return (
        (i % 100) / 50 - 1,
        ((i * 1.3) % 100) / 50 - 1,
      );
    }).listen((value) {
      _gNotifier.value = Offset(value.$1, value.$2);
    });

    // GPS controller and stream
    _gpsController = TextEditingController(text: 'Initializing GPS...');
    _initGps();
  }

  @override
  void dispose() {
    _gSub?.cancel();
    _gNotifier.dispose();
    _positionSub?.cancel();
    _recordingSub?.cancel();
    _gpsController.dispose();
    // previously disposed glow controller; nothing to dispose here
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final sled = widget.sled;

    // Build a ThemeData from the sled colors and wrap the screen so
    // the sled's color scheme is applied to this route only.
    final colorScheme = ColorScheme.light(
      primary: sled.primaryColor,
      secondary: sled.secondaryColor,
    );

    final sledTheme = ThemeData.from(colorScheme: colorScheme).copyWith(
      appBarTheme: Theme.of(context).appBarTheme.copyWith(
        backgroundColor: sled.primaryColor,
        foregroundColor: colorScheme.onPrimary,
      ),
      cardTheme: CardThemeData(
        color: colorScheme.surfaceContainerHighest,
        elevation: 2,
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
      ),
      chipTheme: ChipThemeData(
        backgroundColor: colorScheme.secondaryContainer,
        disabledColor: colorScheme.onSurface.withAlpha((0.12 * 255).round()),
        selectedColor: colorScheme.primary,
        secondarySelectedColor: colorScheme.primaryContainer,
        padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
        labelStyle: TextStyle(color: colorScheme.onSecondary),
        secondaryLabelStyle: TextStyle(color: colorScheme.onPrimary),
        brightness: Brightness.light,
        shape: const StadiumBorder(),
      ),
      elevatedButtonTheme: ElevatedButtonThemeData(
        style: ElevatedButton.styleFrom(
          backgroundColor: colorScheme.primary,
          foregroundColor: colorScheme.onPrimary,
        ),
      ),
      textButtonTheme: TextButtonThemeData(
        style: TextButton.styleFrom(foregroundColor: colorScheme.primary),
      ),
      floatingActionButtonTheme: FloatingActionButtonThemeData(
        backgroundColor: colorScheme.primary,
        foregroundColor: colorScheme.onPrimary,
      ),
      bottomNavigationBarTheme: BottomNavigationBarThemeData(
        backgroundColor: colorScheme.surface,
        selectedItemColor: colorScheme.primary,
        unselectedItemColor: colorScheme.onSurface.withAlpha((0.6 * 255).round()),
      ),
    );

    return Theme(
      data: sledTheme,
      child: Scaffold(
        appBar: AppBar(
          title: Text(sled.name),
        ),
        bottomNavigationBar: BottomNavigationBar(
          currentIndex: _selectedIndex,
          onTap: _onItemTapped,
          items: const [
            BottomNavigationBarItem(
              icon: Icon(Icons.dashboard),
              label: "Overview",
            ),
            BottomNavigationBarItem(
              icon: Icon(Icons.lightbulb),
              label: "LEDs",
            ),
            BottomNavigationBarItem(
              icon: Icon(Icons.sensors),
              label: "Sensors",
            ),
          ],
        ),
        body: Consumer<SettingsProvider>(
          builder: (context, provider, child) {
            final settings = provider.settings;

            return Padding(
              padding: const EdgeInsets.all(16.0),
              child: _buildSelectedTab(context, provider, settings.color, colorScheme),
            );
          },
        ),
      ),
    );
  }

  Widget _buildSelectedTab(
    BuildContext context,
    SettingsProvider provider,
    int color,
    ColorScheme colorScheme,
  ) {
    switch (_selectedIndex) {
      case 0:
        return _buildOverview(context, provider, colorScheme);
      case 1:
        return _buildLedControl(context, provider, color, colorScheme);
      case 2:
        // return const Center(child: Text("Sensors coming soon"));
        return _buildSensorGraphs();
      default:
        return const SizedBox();
    }
  }

  Widget _buildOverview(
    BuildContext context,
    SettingsProvider provider,
    ColorScheme colorScheme,
  ) {
    final sled = widget.sled;

    return SingleChildScrollView(
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              SizedBox(
                height: 200,
                child: Image.asset(
                  sled.imagePath,
                  fit: BoxFit.fitHeight,
                  alignment: Alignment.centerLeft,
                ),
              ),
              const SizedBox(width: 10),
              Expanded(
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    _buildConnectionStatus(provider.isConnected),
                    const SizedBox(height: 8),
                    Card(
                      elevation: 1,
                      shape: RoundedRectangleBorder(
                        borderRadius: BorderRadius.circular(12),
                      ),
                      child: Padding(
                        padding: const EdgeInsets.all(12),
                        child: Column(
                          crossAxisAlignment: CrossAxisAlignment.start,
                          children: [
                            Row(
                              children: [
                                const Icon(Icons.battery_full, size: 20),
                                const SizedBox(width: 8),
                                const Text('Battery', style: TextStyle(fontWeight: FontWeight.w600)),
                                const Spacer(),
                                const Text('80%', style: TextStyle(fontWeight: FontWeight.w700)),
                              ],
                            ),
                            const SizedBox(height: 8),
                            ClipRRect(
                              borderRadius: BorderRadius.circular(8),
                              child: LinearProgressIndicator(
                                value: 0.8,
                                minHeight: 6,
                                  backgroundColor: Theme.of(context).colorScheme.onSurface.withAlpha((0.06 * 255).round()),
                              ),
                            ),
                            const SizedBox(height: 12),
                            Row(
                              children: [
                                const Icon(Icons.place, size: 18),
                                const SizedBox(width: 8),
                                const Text('Total Distance', style: TextStyle(fontWeight: FontWeight.w600)),
                                const Spacer(),
                                const Text('12.5 km', style: TextStyle(fontWeight: FontWeight.w500)),
                              ],
                            ),
                            const SizedBox(height: 8),
                            Row(
                              children: [
                                const Icon(Icons.timer, size: 18),
                                const SizedBox(width: 8),
                                const Text('Total Time', style: TextStyle(fontWeight: FontWeight.w600)),
                                const Spacer(),
                                const Text('45 min', style: TextStyle(fontWeight: FontWeight.w500)),
                              ],
                            ),
                            const SizedBox(height: 8),
                            Row(
                              children: [
                                const Icon(Icons.calendar_today, size: 18),
                                const SizedBox(width: 8),
                                const Text('Last Ride', style: TextStyle(fontWeight: FontWeight.w600)),
                                const Spacer(),
                                const Text('2024-06-01', style: TextStyle(fontWeight: FontWeight.w500)),
                              ],
                            ),
                          ],
                        ),
                      ),
                    ),
                  ],
                ),
              ),
            ],
          ),
          const SizedBox(height: 24),
          _buildSectionHeader('Advanced Settings'),
          const SizedBox(height: 12),
          Card(
            elevation: 2,
            shape: RoundedRectangleBorder(
              borderRadius: BorderRadius.circular(16),
            ),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                _buildSlider(
                  title: 'Long Press Duration',
                  value: provider.settings.longPressDuration.toDouble(),
                  min: 500,
                  max: 3000,
                  divisions: 25,
                  suffix: 'ms',
                  onChanged: (v) => provider.updateLongPressDuration(v.round()),
                  colorScheme: colorScheme,
                  parentTheme: Theme.of(context),
                ),
                const SizedBox(height: 5),
                _buildSlider(
                  title: 'Sideways Threshold Time',
                  value: provider.settings.sidewaysThresholdTime.toDouble(),
                  min: 500,
                  max: 3000,
                  divisions: 25,
                  suffix: 'ms',
                  onChanged: (v) =>
                      provider.updateSidewaysThresholdTime(v.round()),
                  colorScheme: colorScheme,
                  parentTheme: Theme.of(context),
                ),
                const SizedBox(height: 5),
                _buildSlider(
                  title: 'Sideways Acceleration Threshold',
                  value: provider.settings.sidewaysAccelThreshold,
                  min: 0.3,
                  max: 1.5,
                  divisions: 12,
                  suffix: 'g',
                  onChanged: (v) => provider.updateSidewaysAccelThreshold(v),
                  decimals: 2,
                  colorScheme: colorScheme,
                  parentTheme: Theme.of(context),
                ),
              ],
            ),
          ),
          const SizedBox(height: 16),
          _buildSwitchTile(
            title: 'Auto Shutdown',
            subtitle: 'Automatically shut down if the sledge is sideways',
            value: provider.settings.accelAutoShutdown,
            onChanged: provider.updateAccelAutoShutdown,
          ),
          _buildSwitchTile(
            title: 'Auto Startup',
            subtitle: 'Automatically start back up when upright',
            value: provider.settings.accelAutoStartup,
            onChanged: provider.updateAccelAutoStartup,
          ),
        ],
      ),
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

  Widget _buildSlider({
    required String title,
    required double value,
    required double min,
    required double max,
    required int divisions,
    required String suffix,
    required ValueChanged<double> onChanged,
    int decimals = 0,
    required ColorScheme colorScheme,
    required ThemeData parentTheme,
  }) {
    final displayValue = value.clamp(min, max);

    return Padding(
      padding: const EdgeInsets.all(16),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            mainAxisAlignment: MainAxisAlignment.spaceBetween,
            children: [
              Text(
                title,
                style: const TextStyle(fontWeight: FontWeight.w500),
              ),
              InkWell(
                onTap: () => _showSliderDialog(
                  title: title,
                  value: displayValue,
                  min: min,
                  max: max,
                  decimals: decimals,
                  onChanged: onChanged,
                  parentTheme: parentTheme,
                ),
                child: Text(
                  suffix.isNotEmpty ? '${displayValue.toStringAsFixed(decimals)} $suffix' : displayValue.toStringAsFixed(decimals),
                  style: TextStyle(
                    color: colorScheme.primary,
                    fontWeight: FontWeight.bold,
                  ),
                ),
              ),
            ],
          ),
          Builder(builder: (context) {
            Gradient gradient = LinearGradient(colors: [colorScheme.secondary, colorScheme.primary]);

            return ColorBarSlider(
              value: displayValue,
              min: min,
              max: max,
              gradient: gradient,
              onChanged: onChanged,
            );
          }),
        ],
      ),
    );
  }


  void _showSliderDialog({
    required String title,
    required double value,
    required double min,
    required double max,
    required int decimals,
    required ValueChanged<double> onChanged,
    ThemeData? parentTheme,
  }) {
    final controller = TextEditingController(text: value.toStringAsFixed(decimals));

    showDialog<void>(
      context: context,
      builder: (context) {
        final dialogTheme = parentTheme ?? Theme.of(context);
        return Theme(
          data: dialogTheme,
          child: AlertDialog(
          title: Text('Set $title'),
          content: TextField(
            controller: controller,
            keyboardType: const TextInputType.numberWithOptions(decimal: true, signed: true),
            autofocus: true,
            decoration: const InputDecoration(
              border: OutlineInputBorder(),
            ),
          ),
          actions: [
            TextButton(
              onPressed: () => Navigator.of(context).pop(),
              child: const Text('Cancel'),
            ),
            FilledButton(
              onPressed: () {
                final raw = controller.text.replaceAll(',', '.').trim();
                final parsed = double.tryParse(raw);
                if (parsed == null) {
                  Navigator.of(context).pop();
                  return;
                }
                final clamped = parsed.clamp(min, max).toDouble();
                onChanged(clamped);
                Navigator.of(context).pop();
              },
              child: const Text('Set'),
            ),
          ],
          ),
        );
      },
    );
  }

  Widget _buildSwitchTile({
    required String title,
    required String subtitle,
    required bool value,
    required ValueChanged<bool> onChanged,
  }) {
    return Card(
      elevation: 2,
      margin: const EdgeInsets.only(bottom: 12),
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
      child: SwitchListTile(
        title: Text(title, style: const TextStyle(fontWeight: FontWeight.w500)),
        subtitle: Text(
          subtitle,
          style: TextStyle(color: Theme.of(context).colorScheme.outline),
        ),
        value: value,
        onChanged: onChanged,
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
      ),
    );
  }

  Widget _buildLedControl(
    BuildContext context,
    SettingsProvider provider,
    int color,
    ColorScheme colorScheme,
  ) {
    return SingleChildScrollView(
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [

          /// FRONTLIGHT SECTION
          _buildSectionHeader('Frontlight'),
          // const SizedBox(height: 5),

          Padding(
            padding: const EdgeInsets.all(16),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Row(
                  crossAxisAlignment: CrossAxisAlignment.center,
                  children: [
                    // Left mode value (Mode 1) - fixed width and tappable
                    InkWell(
                      onTap: () => _showSliderDialog(
                        title: 'Brightness / Mode',
                        value: frontlightValue,
                        min: -100,
                        max: 100,
                        decimals: 0,
                        onChanged: (v) => setState(() => frontlightValue = v),
                        parentTheme: Theme.of(context),
                      ),
                      child: SizedBox(
                        width: 72,
                        child: Center(
                          child: Text(
                            frontlightValue < 0 ? '${frontlightValue.abs().toStringAsFixed(0)} %' : '0 %',
                            textAlign: TextAlign.center,
                            style: TextStyle(
                              color: frontlightValue < 0 ? Theme.of(context).colorScheme.primary : Theme.of(context).colorScheme.onSurfaceVariant,
                              fontWeight: frontlightValue < 0 ? FontWeight.bold : FontWeight.normal,
                            ),
                          ),
                        ),
                      ),
                    ),

                    // Centered title (takes remaining space)
                    Expanded(
                      child: Center(
                        child: Text(
                          'Brightness',
                          style: const TextStyle(fontWeight: FontWeight.w500),
                        ),
                      ),
                    ),

                    // Right mode value (Mode 2) - fixed width and tappable
                    InkWell(
                      onTap: () => _showSliderDialog(
                        title: 'Brightness / Mode',
                        value: frontlightValue,
                        min: -100,
                        max: 100,
                        decimals: 0,
                        onChanged: (v) => setState(() => frontlightValue = v),
                        parentTheme: Theme.of(context),
                      ),
                      child: SizedBox(
                        width: 72,
                        child: Center(
                          child: Text(
                            frontlightValue > 0 ? '${frontlightValue.toStringAsFixed(0)} %' : '0 %',
                            textAlign: TextAlign.center,
                            style: TextStyle(
                              color: frontlightValue > 0 ? Theme.of(context).colorScheme.primary : Theme.of(context).colorScheme.onSurfaceVariant,
                              fontWeight: frontlightValue > 0 ? FontWeight.bold : FontWeight.normal,
                            ),
                          ),
                        ),
                      ),
                    ),
                  ],
                ),
                const SizedBox(height: 8),
                SplitColorBarSlider(
                  value: frontlightValue,
                  min: -100,
                  max: 100,
                  gradient: frontlightValue >= 0
                      ? LinearGradient(colors: [colorScheme.primaryContainer, colorScheme.primary])
                      : LinearGradient(colors: [colorScheme.secondary, colorScheme.primary]),
                  showZeroMarker: true,
                  onChanged: (val) => setState(() => frontlightValue = val),
                ),
              ],
            ),
          ),

          Row(
            mainAxisAlignment: MainAxisAlignment.spaceBetween,
            children: const [
              Text("Mode 1"),
              Text("OFF"),
              Text("Mode 2"),
            ],
          ),

          const SizedBox(height: 40),

          /// RGB STRIP SECTION
          _buildSectionHeader('RGB Strip'),
          // const SizedBox(height: 5),

          _buildSlider(
            title: 'Brightness',
            value: provider.settings.brightness.toDouble(),
            min: 0,
            max: 100,
            divisions: 100,
            suffix: '%',
            onChanged: (v) =>
                provider.updateBrightness(v.round()),
            colorScheme: colorScheme,
            parentTheme: Theme.of(context),
          ),

          _buildSlider(
            title: 'Effect Speed',
            value: provider.settings.effectSpeed.toDouble(),
            min: 0,
            max: 100,
            divisions: 100,
            suffix: '%',
            onChanged: (v) =>
                provider.updateEffectSpeed(v.round()),
            colorScheme: colorScheme,
            parentTheme: Theme.of(context),
          ),
          
          const SizedBox(height: 16),

          _buildEffectCard(context, provider),

          const SizedBox(height: 24),

          _buildColorWheel(context, provider),
        ],
      ),
    );
  }

  Widget _buildColorWheel(
    BuildContext context,
    SettingsProvider provider,
  ) {
    return Card(
      elevation: 2,
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const Text(
              'LED Color',
              style: TextStyle(fontWeight: FontWeight.w500),
            ),
            const SizedBox(height: 12),
            Row(
              children: [
                GestureDetector(
                  onTap: () => _showColorPicker(context, provider),
                  child: Container(
                    width: 48,
                    height: 48,
                    decoration: BoxDecoration(
                      color: Color(provider.settings.color),
                      shape: BoxShape.circle,
                      border: Border.all(color: Colors.white, width: 2),
                      boxShadow: [
                        BoxShadow(
                          color: Colors.black26,
                          blurRadius: 4,
                          offset: const Offset(0, 2),
                        ),
                      ],
                    ),
                  ),
                ),
                const SizedBox(width: 12),
                Expanded(
                  child: ElevatedButton(
                    onPressed: () => _showColorPicker(context, provider),
                    child: const Text('Change Color'),
                  ),
                ),
              ],
            ),
          ],
        ),
      ),
    );
  }

  void _showColorPicker(
    BuildContext context,
    SettingsProvider provider,
  ) {
    Color pickedColor = Color(provider.settings.color);
    showDialog<void>(
      context: context,
      builder: (context) {
        return StatefulBuilder(
          builder: (context, setState) {
            return AlertDialog(
              title: const Text('Pick a color'),
              content: SingleChildScrollView(
                child: Column(
                  mainAxisSize: MainAxisSize.min,
                  children: [
                    ColorWheel(
                      initialColor: pickedColor,
                      onChanged: (c) => setState(() => pickedColor = c),
                      size: 260,
                    ),
                  ],
                ),
              ),
              actions: [
                TextButton(
                  onPressed: () => Navigator.pop(context),
                  child: const Text('Cancel'),
                ),
                FilledButton(
                  onPressed: () {
                    provider.updateColor(pickedColor.toARGB32());
                    Navigator.pop(context);
                  },
                  child: const Text('Select'),
                ),
              ],
            );
          },
        );
      },
    );
  }

  Widget _buildEffectCard(BuildContext context, SettingsProvider provider) {
    final currentEffectName = provider.settings.effect;
    final currentName = Settings.effects[currentEffectName]?.displayName ?? currentEffectName;

    return Card(
      elevation: 2,
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const Text('Effect', style: TextStyle(fontWeight: FontWeight.w500)),
            const SizedBox(height: 12),
            EffectBorder(
              effectName: currentEffectName,
              selected: true,
              baseColor: Color(provider.settings.color),
              segments: 80,
              height: 44,
              child: Material(
                color: Colors.transparent,
                child: InkWell(
                  borderRadius: BorderRadius.circular(12),
                  onTap: () => _showEffectPicker(context, provider, currentEffectName),
                  child: Container(
                    width: double.infinity,
                    padding: const EdgeInsets.symmetric(vertical: 8, horizontal: 16),
                    child: Text(currentName, style: const TextStyle(fontWeight: FontWeight.w600)),
                  ),
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }

  void _showEffectPicker(BuildContext context, SettingsProvider provider, String currentEffectName) {
    final names = Settings.effectNames;
    final display = Settings.effectDisplayNames;

    showDialog<void>(
      context: context,
      builder: (context) {
        int selectedIdx = names.indexOf(currentEffectName);
        if (selectedIdx < 0) selectedIdx = 0;

        return StatefulBuilder(
          builder: (context, setState) {
            return AlertDialog(
              title: const Text('Pick an effect'),
              content: SizedBox(
                width: 360,
                height: 320,
                child: ListView.separated(
                  shrinkWrap: true,
                  itemCount: names.length,
                  separatorBuilder: (_, _) => const SizedBox(height: 8),
                  itemBuilder: (context, idx) {
                    final name = names[idx];
                    final label = display[idx];
                    final selected = idx == selectedIdx;

                    return EffectBorder(
                      effectName: name,
                      selected: selected,
                      baseColor: Color(provider.settings.color),
                      segments: 80,
                      height: 44,
                      child: Material(
                        color: Colors.transparent,
                        child: InkWell(
                          borderRadius: BorderRadius.circular(12),
                          onTap: () => setState(() => selectedIdx = idx),
                          child: Padding(
                            padding: const EdgeInsets.symmetric(horizontal: 12),
                            child: Align(
                              alignment: Alignment.centerLeft,
                                child: Text(
                                label,
                                style: TextStyle(
                                  fontWeight: FontWeight.w600,
                                  color: Colors.black,
                                ),
                              ),
                            ),
                          ),
                        ),
                      ),
                    );
                  },
                ),
              ),
              actions: [
                TextButton(
                  onPressed: () => Navigator.pop(context),
                  child: const Text('Cancel'),
                ),
                FilledButton(
                  onPressed: () {
                    provider.updateEffect(names[selectedIdx]);
                    Navigator.pop(context);
                  },
                  child: const Text('Select'),
                ),
              ],
            );
          },
        );
      },
    );
  }

  Widget _buildSensorGraphs() {
    return Column(
      children: [
        Card(
          elevation: 2,
          shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
          child: Padding(
            padding: const EdgeInsets.all(12),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                const Text('GPS Info', style: TextStyle(fontWeight: FontWeight.w600)),
                const SizedBox(height: 8),
                TextField(
                  controller: _gpsController,
                  readOnly: true,
                  maxLines: 5,
                  decoration: const InputDecoration(border: OutlineInputBorder()),
                ),
              ],
            ),
          ),
        ),
        const SizedBox(height: 8),
        Row(
          children: [
            Expanded(
              child: ElevatedButton.icon(
                icon: Icon(_isRecording ? Icons.stop : Icons.play_arrow),
                label: Text(_isRecording ? 'Stop & Save' : 'Start Recording'),
                onPressed: () => _toggleRecording(),
              ),
            ),
            const SizedBox(width: 8),
            ElevatedButton(
              onPressed: () {
                // quick clear recorded points
                _recordedPoints.clear();
                ScaffoldMessenger.of(context).showSnackBar(const SnackBar(content: Text('Recorded buffer cleared')));
              },
              child: const Text('Clear'),
            ),
          ],
        ),
        const SizedBox(height: 12),
        const Text(
          "Live Acceleration (G-Plot)",
          style: TextStyle(fontWeight: FontWeight.w500),
        ),
        const SizedBox(height: 12),
        GPlot(gx: 0.0, gy: 0.0, valueListenable: _gNotifier),
      ],
    );
  }

  void _toggleRecording() async {
    if (_isRecording) {
      // stop
      // Capture objects that depend on BuildContext before any awaits.
      final ridesProv = context.read<RidesProvider>();
      final messenger = ScaffoldMessenger.of(context);

      // Stop background service if running
      try {
        await fg_service.stopForegroundRecording();
      } catch (_) {}
      if (_recordingSub != null) await _recordingSub!.cancel();
      _recordingSub = null;
      setState(() => _isRecording = false);

      if (!mounted) return;

      if (_recordedPoints.isEmpty) {
        // If foreground service recorded points, try loading them
        final bgPoints = await fg_service.loadBackgroundRecordedPoints();
        if (bgPoints.isNotEmpty) {
          _recordedPoints.addAll(bgPoints);
        }

        if (_recordedPoints.isEmpty) {
          messenger.showSnackBar(const SnackBar(content: Text('No GPS points recorded')));
          return;
        }
      }

      final startTs = _recordedPoints.first.timestamp;
      final ride = Ride(id: ridesProv.rides.length, points: List<GPSPoint>.from(_recordedPoints), startTime: DateTime.fromMillisecondsSinceEpoch(startTs));
      await ridesProv.addRide(ride);
      if (!mounted) return;
      ScaffoldMessenger.of(context).showSnackBar(const SnackBar(content: Text('Ride saved')));
      _recordedPoints.clear();
    } else {
      // start
      _recordedPoints.clear();
      setState(() => _isRecording = true);

      try {
        // Ensure background permission before starting native foreground service.
        bool canBg = false;
        try {
          canBg = await _ensureBackgroundPermission();
        } catch (_) {
          canBg = false;
        }

        if (canBg) {
          try {
            await fg_service.startForegroundRecording(title: 'Rodl recording', text: 'Recording ride');
          } catch (e) {
            if (kDebugMode) print('Could not start foreground service: $e');
          }
        } else {
          // Ask the user to open app settings to grant background location.
          final open = await showDialog<bool>(
            context: context,
            builder: (dctx) => AlertDialog(
              title: const Text('Background location required'),
              content: const Text('To continue recording while the app is in the background, please grant "Always allow" location permission in app settings.'),
              actions: [
                TextButton(onPressed: () => Navigator.of(dctx).pop(false), child: const Text('Cancel')),
                TextButton(onPressed: () => Navigator.of(dctx).pop(true), child: const Text('Open settings')),
              ],
            ),
          );
          if (open == true) {
            await Geolocator.openAppSettings();
          }
          if (kDebugMode) print('Background permission not granted - native service skipped');
        }

        _recordingSub = Geolocator.getPositionStream(
          locationSettings: const LocationSettings(accuracy: LocationAccuracy.best, distanceFilter: 1),
        ).listen((p) {
          final ts = p.timestamp.millisecondsSinceEpoch;
          final point = GPSPoint(
            timestamp: ts,
            lat: p.latitude,
            lon: p.longitude,
            speedKmh: p.speed * 3.6,
            altM: p.altitude,
            sats: 0,
            hdop: p.accuracy,
            age: 0,
          );
          _recordedPoints.add(point);
          _gpsController.text = _formatPosition(p);
        }, onError: (e) {
          _gpsController.text = 'Recording stream error: $e';
        });
      } catch (e) {
        setState(() => _isRecording = false);
        _gpsController.text = 'Failed to start recording: $e';
      }
    }
  }

  Future<void> _initGps() async {
    try {
      _gpsController.text = 'Checking location services...';
      final serviceEnabled = await Geolocator.isLocationServiceEnabled();
      if (!serviceEnabled) {
        _gpsController.text = 'Location services are disabled.';
        return;
      }

      _gpsController.text = 'Checking permissions...';
      LocationPermission permission = await Geolocator.checkPermission();
      if (permission == LocationPermission.denied) {
        _gpsController.text = 'Requesting location permission...';
        permission = await Geolocator.requestPermission();
        if (permission == LocationPermission.denied) {
          _gpsController.text = 'Location permissions are denied.';
          return;
        }
      }

      if (permission == LocationPermission.deniedForever) {
        _gpsController.text = 'Location permissions are permanently denied.';
        return;
      }

      _gpsController.text = 'Fetching current location...';
      final pos = await Geolocator.getCurrentPosition(
        locationSettings: const LocationSettings(accuracy: LocationAccuracy.best),
      );
      _gpsController.text = _formatPosition(pos);

      _gpsController.text = 'Subscribing to location updates...';
      _positionSub = Geolocator.getPositionStream(
        locationSettings: const LocationSettings(accuracy: LocationAccuracy.best, distanceFilter: 1),
      ).listen((p) {
        _gpsController.text = _formatPosition(p);
      }, onError: (e) {
        _gpsController.text = 'GPS stream error: $e';
      });
    } catch (e, st) {
      _gpsController.text = 'GPS error: $e';
      // also log to console for more context when running
      // ignore: avoid_print
      print('GPS init error: $e\n$st');
    }
  }

  Future<bool> _ensureBackgroundPermission() async {
    try {
      final serviceEnabled = await Geolocator.isLocationServiceEnabled();
      if (!serviceEnabled) {
        _gpsController.text = 'Location services are disabled.';
        return false;
      }

      LocationPermission permission = await Geolocator.checkPermission();
      if (permission == LocationPermission.denied) {
        permission = await Geolocator.requestPermission();
      }

      if (permission == LocationPermission.deniedForever) {
        _gpsController.text = 'Location permissions are permanently denied.';
        return false;
      }

      if (permission == LocationPermission.whileInUse) {
        // Attempt to request 'always' permission; on some devices this will
        // still require the user to grant it manually in app settings.
        permission = await Geolocator.requestPermission();
      }

      return permission == LocationPermission.always;
    } catch (e) {
      _gpsController.text = 'Permission check failed: $e';
      return false;
    }
  }

  String _formatPosition(Position p) {
    final time = p.timestamp.toIso8601String();
    return 'Lat: ${p.latitude.toStringAsFixed(6)}\nLng: ${p.longitude.toStringAsFixed(6)}\nAlt: ${p.altitude.toStringAsFixed(2)} m\nSpeed: ${p.speed.toStringAsFixed(2)} m/s\nTime: $time';
  }

}
