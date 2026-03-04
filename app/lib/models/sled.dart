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

  factory Sled.fromJson(Map<String, dynamic> json) {
    Color parseColor(dynamic v) {
      if (v is int) return Color(v);
      if (v is String) {
        var s = v.replaceFirst('#', '');
        if (s.length == 6) s = 'FF$s';
        final val = int.parse(s, radix: 16);
        return Color(val);
      }
      return const Color(0xFF000000);
    }

    return Sled(
      id: json['id'] as String,
      name: json['name'] as String,
      imagePath: json['imagePath'] as String,
      primaryColor: parseColor(json['primaryColor']),
      secondaryColor: parseColor(json['secondaryColor']),
      // `isFavorite` is an app-local preference and must not be derived from server data.
      isFavorite: false,
    );
  }

  Map<String, dynamic> toJson() {
    String toHex(Color c) => '#${c.toARGB32().toRadixString(16).padLeft(8, '0').toUpperCase()}';

    return {
      'id': id,
      'name': name,
      'imagePath': imagePath,
      'primaryColor': toHex(primaryColor),
      'secondaryColor': toHex(secondaryColor),
      // do NOT include `isFavorite` here because it's an app-local flag and
      // should not be sent back to the server.
    };
  }
}