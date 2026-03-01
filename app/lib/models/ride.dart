import 'dart:math';
import 'package:intl/intl.dart';
import 'gps_point.dart';

class Ride {
  final int id;
  final List<GPSPoint> points;
  final DateTime startTime;
  final String? name;

  Ride({required this.id, required this.points, required this.startTime, this.name});

  DateTime get date => DateTime(startTime.year, startTime.month, startTime.day);

  String get formattedDate => DateFormat('EEEE, MMMM d, yyyy').format(date);

  int get durationSeconds {
    if (points.isEmpty) return 0;
    return ((points.last.timestamp - points.first.timestamp) / 1000).round();
  }

  String get formattedDuration {
    final hours = durationSeconds ~/ 3600;
    final minutes = (durationSeconds % 3600) ~/ 60;
    final seconds = durationSeconds % 60;
    if (hours > 0) {
      return '${hours}h ${minutes}m ${seconds}s';
    } else if (minutes > 0) {
      return '${minutes}m ${seconds}s';
    }
    return '${seconds}s';
  }

  double get maxSpeed {
    if (points.isEmpty) return 0;
    return points.map((p) => p.speedKmh).reduce(max);
  }

  double get avgSpeed {
    if (points.isEmpty) return 0;
    return points.map((p) => p.speedKmh).reduce(max) / 2;
  }

  double get totalDistanceKm {
    if (points.length < 2) return 0;
    double total = 0;
    for (int i = 1; i < points.length; i++) {
      total += _haversineDistance(
        points[i - 1].lat,
        points[i - 1].lon,
        points[i].lat,
        points[i].lon,
      );
    }
    return total;
  }

  double _haversineDistance(
    double lat1,
    double lon1,
    double lat2,
    double lon2,
  ) {
    const double earthRadiusKm = 6371;
    final dLat = _toRadians(lat2 - lat1);
    final dLon = _toRadians(lon2 - lon1);
    final a =
        sin(dLat / 2) * sin(dLat / 2) +
        cos(_toRadians(lat1)) *
            cos(_toRadians(lat2)) *
            sin(dLon / 2) *
            sin(dLon / 2);
    final c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return earthRadiusKm * c;
  }

  double _toRadians(double degrees) => degrees * pi / 180;

  Map<String, dynamic> toJson() => {
        'id': id,
        'startTime': startTime.millisecondsSinceEpoch,
        'points': points.map((p) => p.toJson()).toList(),
        'name': name,
      };

  factory Ride.fromJson(Map<String, dynamic> json) {
    final pts =
        (json['points'] as List).map((e) => GPSPoint.fromJson(Map<String, dynamic>.from(e))).toList();
    return Ride(
      id: json['id'] as int,
      points: pts,
      startTime: DateTime.fromMillisecondsSinceEpoch(json['startTime'] as int),
      name: json['name'] as String?,
    );
  }
}
