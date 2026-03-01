import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:flutter_colorpicker/flutter_colorpicker.dart';
import '../providers/settings_provider.dart';
import '../models/settings.dart';

class SettingsScreen extends StatefulWidget {
  const SettingsScreen({super.key});

  @override
  State<SettingsScreen> createState() => _SettingsScreenState();
}

class _SettingsScreenState extends State<SettingsScreen> {
  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addPostFrameCallback((_) {
      context.read<SettingsProvider>().connect();
    });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('Settings'), centerTitle: true),
      body: Consumer<SettingsProvider>(
        builder: (context, provider, child) {
          final settings = provider.settings;
          final isConnected = provider.isConnected;

          return ListView(
            padding: const EdgeInsets.all(16),
            children: [
              _buildConnectionStatus(isConnected),
              const SizedBox(height: 24),
              _buildSectionHeader('LED Settings'),
              const SizedBox(height: 12),
              _buildColorPicker(context, provider, settings.color),
              const SizedBox(height: 16),
              _buildSliderTile(
                title: 'Brightness',
                value: settings.brightness.toDouble(),
                min: 0,
                max: 255,
                divisions: 255,
                suffix: '',
                onChanged: (v) => provider.updateBrightness(v.round()),
              ),
              const SizedBox(height: 16),
              _buildEffectDropdown(context, provider, settings),
              const SizedBox(height: 16),
              _buildSliderTile(
                title: 'Effect Speed',
                value: settings.effectSpeed.toDouble(),
                min: 100,
                max: 10000,
                divisions: 99,
                suffix: '',
                onChanged: (v) => provider.updateEffectSpeed(v.round()),
              ),
              const SizedBox(height: 32),
              _buildSectionHeader('Advanced Settings'),
              const SizedBox(height: 12),
              _buildSliderTile(
                title: 'Long Press Duration',
                value: settings.longPressDuration.toDouble(),
                min: 500,
                max: 3000,
                divisions: 25,
                suffix: 'ms',
                onChanged: (v) => provider.updateLongPressDuration(v.round()),
              ),
              const SizedBox(height: 16),
              _buildSliderTile(
                title: 'Sideways Threshold Time',
                value: settings.sidewaysThresholdTime.toDouble(),
                min: 500,
                max: 3000,
                divisions: 25,
                suffix: 'ms',
                onChanged: (v) =>
                    provider.updateSidewaysThresholdTime(v.round()),
              ),
              const SizedBox(height: 16),
              _buildSliderTile(
                title: 'Sideways Acceleration Threshold',
                value: settings.sidewaysAccelThreshold,
                min: 0.3,
                max: 1.5,
                divisions: 12,
                suffix: 'g',
                onChanged: (v) => provider.updateSidewaysAccelThreshold(v),
              ),
              const SizedBox(height: 16),
              _buildSwitchTile(
                title: 'Auto Shutdown',
                subtitle: 'Automatically shut down if the sledge is sideways',
                value: settings.accelAutoShutdown,
                onChanged: provider.updateAccelAutoShutdown,
              ),
              _buildSwitchTile(
                title: 'Auto Startup',
                subtitle: 'Automatically start back up when upright',
                value: settings.accelAutoStartup,
                onChanged: provider.updateAccelAutoStartup,
              ),
            ],
          );
        },
      ),
      endDrawerEnableOpenDragGesture: true,
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

  Widget _buildColorPicker(
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

  Widget _buildSliderTile({
    required String title,
    required double value,
    required double min,
    required double max,
    required int divisions,
    required String suffix,
    required ValueChanged<double> onChanged,
  }) {
    return Card(
      elevation: 2,
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
      child: Padding(
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
      ),
    );
  }

  Widget _buildEffectDropdown(
    BuildContext context,
    SettingsProvider provider,
    Settings settings,
  ) {
    final names = Settings.effectNames;
    final display = Settings.effectDisplayNames;
    final currentName = settings.effect;
    return Card(
      elevation: 2,
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const Text(
              'LED Effect',
              style: TextStyle(fontWeight: FontWeight.w500),
            ),
            const SizedBox(height: 12),
            DropdownButtonFormField<String>(
              initialValue: currentName,
              decoration: InputDecoration(
                border: OutlineInputBorder(
                  borderRadius: BorderRadius.circular(12),
                ),
                contentPadding: const EdgeInsets.symmetric(
                  horizontal: 16,
                  vertical: 12,
                ),
              ),
              items: names.asMap().entries.map((e) {
                return DropdownMenuItem(value: e.value, child: Text(display[e.key]));
              }).toList(),
              onChanged: (v) {
                if (v != null) provider.updateEffect(v);
              },
            ),
          ],
        ),
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
}
