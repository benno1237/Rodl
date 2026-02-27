import 'package:flutter/material.dart';
import 'package:rodl/models/settings.dart';
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
    Stream.periodic(const Duration(milliseconds: 100), (i) {
      return (
        (i % 100) / 50 - 1,
        ((i * 1.3) % 100) / 50 - 1,
      );
    }).listen((value) {
      setState(() {
        gx = value.$1;
        gy = value.$2;
      });
    });
  }

  @override
  Widget build(BuildContext context) {
    final sled = widget.sled; // <-- IMPORTANT change

    return Scaffold(
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
        child: _buildSelectedTab(context, provider, settings.color)
      );
    },
      ),
    );
  }

  Widget _buildSelectedTab(
    BuildContext context,
    SettingsProvider provider,
    int color,
  ) {
    switch (_selectedIndex) {
      case 0:
        return _buildOverview(context, provider);
      case 1:
        return _buildLedControl(context, provider, color);
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
  ) {
    final sled = widget.sled;

    return SingleChildScrollView(
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Image.asset(
                sled.imagePath,
                height: 200,
                fit: BoxFit.contain,
              ),
              const SizedBox(width: 16),
              _buildConnectionStatus(provider.isConnected),
              const Expanded(
                child: Text(
                  "Battery: 80%\n"
                  "Total Distance: 12.5 km\n"
                  "Total Time: 45 minutes\n"
                  "Last Ride: 2024-06-01\n",
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
                    ),
                    const SizedBox(height: 10),
                    _buildSlider(
                      title: 'Sideways Threshold Time',
                      value: provider.settings.sidewaysThresholdTime.toDouble(),
                      min: 500,
                      max: 3000,
                      divisions: 25,
                      suffix: 'ms',
                      onChanged: (v) =>
                          provider.updateSidewaysThresholdTime(v.round()),
                    ),
                    const SizedBox(height: 10),
                    _buildSlider(
                      title: 'Sideways Acceleration Threshold',
                      value: provider.settings.sidewaysAccelThreshold,
                      min: 0.3,
                      max: 1.5,
                      divisions: 12,
                      suffix: 'g',
                      onChanged: (v) => provider.updateSidewaysAccelThreshold(v),
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
  }) {
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
                suffix.isNotEmpty
                    ? '${value.toStringAsFixed(0)} $suffix'
                    : value.toStringAsFixed(0),
                style: TextStyle(
                  color: Theme.of(context).colorScheme.primary,
                  fontWeight: FontWeight.bold,
                ),
              ),
            ],
          ),
          Slider(
            value: value.clamp(min, max),
            min: min,
            max: max,
            divisions: divisions,
            onChanged: onChanged,
          ),
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
              ? LinearGradient(colors: [Colors.green, Colors.lightGreen])
              : LinearGradient(colors: [Colors.orange, Colors.red]),
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
              colors: [Colors.blue, Colors.cyan],
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
              provider.updateColor(pickedColor.value);
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
        GPlot(gx: gx, gy: gy),
      ],
    );
  }

}
