import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'services/tile_cache.dart';
import 'providers/rides_provider.dart';
import 'providers/settings_provider.dart';
import 'screens/home_screen.dart';
import 'data/sleds.dart';

Future<void> main() async {
  WidgetsFlutterBinding.ensureInitialized();
  await CachedNetworkTileProvider.init();
  await initSleds();
  runApp(const RodlApp());
}

class RodlApp extends StatelessWidget {
  const RodlApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MultiProvider(
      providers: [
        ChangeNotifierProvider(create: (_) => RidesProvider()),
        ChangeNotifierProvider(create: (_) => SettingsProvider()),
      ],
      child: MaterialApp(
        title: 'Rodl',
        debugShowCheckedModeBanner: false,
        theme: ThemeData(
          colorScheme: ColorScheme.fromSeed(
            seedColor: const Color(0xFF7530AA),
            brightness: Brightness.light,
          ),
          useMaterial3: true,
          appBarTheme: const AppBarTheme(centerTitle: true, elevation: 0),
          cardTheme: CardThemeData(
            elevation: 2,
            shape: RoundedRectangleBorder(
              borderRadius: BorderRadius.circular(16),
            ),
          ),
        ),
        home: const HomeScreen(),
      ),
    );
  }
}
