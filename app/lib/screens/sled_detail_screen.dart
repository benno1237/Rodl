import 'dart:async';
import 'package:flutter/material.dart';
import '../models/sled.dart';
import '../widgets/g_plot.dart';
import '../widgets/color_bar_slider.dart';
import 'package:flutter_colorpicker/flutter_colorpicker.dart';
import 'package:provider/provider.dart';
import '../providers/settings_provider.dart';

class SledDetailScreen extends StatefulWidget {
  final Sled sled;

  const SledDetailScreen({super.key, required this.sled});

  @override
  State<SledDetailScreen> createState() => _SledDetailScreenState();
}

class _SledDetailScreenState extends State<SledDetailScreen> {
  double gx = 0.0;
  double gy = 0.0;

  late final ValueNotifier<Offset> _gNotifier;
  StreamSubscription? _gSub;

  @override
  void setState(VoidCallback fn) {
    if (!mounted) return;
    super.setState(fn);
  }

  int _selectedIndex = 0;

  // Frontlight
  double frontlightValue = 0; // -100 to +100

  // RGB strip
  double stripBrightness = 50;
  String selectedEffect = "Static";
  Color selectedColor = Colors.blue;

  void _onItemTapped(int index) {
    setState(() {
      _selectedIndex = index;
    });
  }

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addPostFrameCallback((_) {
      context.read<SettingsProvider>().connect();
    });

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
  }

  @override
  void dispose() {
    _gSub?.cancel();
    _gNotifier.dispose();
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
        disabledColor: colorScheme.onSurface.withOpacity(0.12),
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
        unselectedItemColor: colorScheme.onSurface.withOpacity(0.6),
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
                                backgroundColor: Theme.of(context).colorScheme.onSurface.withOpacity(0.06),
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
              Text(
                suffix.isNotEmpty ? '${displayValue.toStringAsFixed(decimals)} $suffix' : displayValue.toStringAsFixed(decimals),
                style: TextStyle(
                  color: colorScheme.primary,
                  fontWeight: FontWeight.bold,
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
          Text("Frontlight", style: Theme.of(context).textTheme.titleLarge),
          const SizedBox(height: 16),

          Text("Brightness / Mode"),
          ColorBarSlider(
            value: frontlightValue,
            min: -100,
            max: 100,
            gradient: frontlightValue >= 0
                ? LinearGradient(colors: [colorScheme.primaryContainer, colorScheme.primary])
                : LinearGradient(colors: [colorScheme.secondary, colorScheme.primary]),
            onChanged: (val) {
              setState(() {
                frontlightValue = val;
              });
            },
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
          Text("RGB Strip", style: Theme.of(context).textTheme.titleLarge),
          const SizedBox(height: 16),

          Text("Brightness"),
          ColorBarSlider(
            value: stripBrightness,
            min: 0,
            max: 100,
            gradient: LinearGradient(
              colors: [colorScheme.primary, colorScheme.secondary],
            ),
            onChanged: (val) {
              setState(() {
                stripBrightness = val;
              });
            },
          ),

          const SizedBox(height: 16),

          Text("Effect"),
          DropdownButton<String>(
            value: selectedEffect,
            isExpanded: true,
            items: [
              "Static",
              "Breathing",
              "Rainbow",
              "Police",
              "Strobe"
            ].map((effect) {
              return DropdownMenuItem(
                value: effect,
                child: Text(effect),
              );
            }).toList(),
            onChanged: (value) {
              setState(() {
                selectedEffect = value!;
              });
            },
          ),

          const SizedBox(height: 24),

          _buildColorWheel(context, provider, color),
        ],
      ),
    );
  }

  Widget _buildColorWheel(
    BuildContext context,
    SettingsProvider provider,
    int color,
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
                  onTap: () => _showColorPicker(context, provider, color),
                  child: Container(
                    width: 48,
                    height: 48,
                    decoration: BoxDecoration(
                      color: Color(color),
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
                    onPressed: () => _showColorPicker(context, provider, color),
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
    int currentColor,
  ) {
    Color pickedColor = Color(currentColor);
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: const Text('Pick a color'),
        content: SingleChildScrollView(
          child: ColorPicker(
            pickerColor: pickedColor,
            onColorChanged: (c) => pickedColor = c,
            enableAlpha: false,
            labelTypes: const [],
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
      ),
    );
  }

  Widget _buildSensorGraphs() {
    return Column(
      children: [
        const Text(
          "Live Acceleration (G-Plot)",
          style: TextStyle(fontWeight: FontWeight.w500),
        ),
        const SizedBox(height: 12),
        GPlot(gx: 0.0, gy: 0.0, valueListenable: _gNotifier),
      ],
    );
  }

}
