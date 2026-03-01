import 'package:flutter/material.dart';

class Sled {
  final String id;
  final String name;
  final String imagePath;
  final Color primaryColor;
  final Color secondaryColor;
  final bool isFavorite;

  Sled({
    required this.id,
    required this.name,
    required this.imagePath,
    required this.primaryColor,
    required this.secondaryColor,
    this.isFavorite = false,
  });
}