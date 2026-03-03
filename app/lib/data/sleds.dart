import 'package:flutter/material.dart';
import '../models/sled.dart';

final List<Sled> baseSleds = [
  Sled(
    id: "SLED-001",
    name: "Red Lightning",
    imagePath: "assets/images/Rodel_rot.png",
    primaryColor: const Color(0xFFE53935),
    secondaryColor: const Color(0xFFFFCDD2),
  ),
  Sled(
    id: "SLED-002",
    name: "Blue Thunder",
    imagePath: "assets/images/Rodel_blau.png",
    primaryColor: const Color(0xFF1E88E5),
    secondaryColor: const Color(0xFFBBDEFB),
  ),
];

final Map<String, Sled> sledsById = {for (var s in baseSleds) s.id: s};
