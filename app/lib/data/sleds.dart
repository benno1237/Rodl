import 'dart:convert';

import 'package:flutter/services.dart';

import '../models/sled.dart';

class _SledsStore {
  static List<Sled> baseSleds = [];
  static Map<String, Sled> sledsById = {};

  static Future<void> init() async {
    try {
      final raw = await rootBundle.loadString('assets/data/sleds.json');
      final list = json.decode(raw) as List<dynamic>;
      baseSleds = list.map((e) => Sled.fromJson(e as Map<String, dynamic>)).toList();
      sledsById = {for (var s in baseSleds) s.id: s};
    } catch (e) {
      baseSleds = [];
      sledsById = {};
    }
  }
}

/// Public accessors kept for backward compatibility.
List<Sled> get baseSleds => _SledsStore.baseSleds;
Map<String, Sled> get sledsById => _SledsStore.sledsById;

/// Initialize sleds from `assets/data/sleds.json`.
Future<void> initSleds() => _SledsStore.init();
