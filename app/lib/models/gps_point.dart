class GPSPoint {
  final int timestamp;
  final double lat;
  final double lon;
  final double speedKmh;
  final double altM;
  final int sats;
  final double hdop;
  final int age;
  final double acceleration;

  GPSPoint({
    required this.timestamp,
    required this.lat,
    required this.lon,
    required this.speedKmh,
    required this.altM,
    required this.sats,
    required this.hdop,
    required this.age,
    this.acceleration = 0.0,
  });

  GPSPoint copyWith({
    int? timestamp,
    double? lat,
    double? lon,
    double? speedKmh,
    double? altM,
    int? sats,
    double? hdop,
    int? age,
    double? acceleration,
  }) {
    return GPSPoint(
      timestamp: timestamp ?? this.timestamp,
      lat: lat ?? this.lat,
      lon: lon ?? this.lon,
      speedKmh: speedKmh ?? this.speedKmh,
      altM: altM ?? this.altM,
      sats: sats ?? this.sats,
      hdop: hdop ?? this.hdop,
      age: age ?? this.age,
      acceleration: acceleration ?? this.acceleration,
    );
  }

  Map<String, dynamic> toJson() => {
        'timestamp': timestamp,
        'lat': lat,
        'lon': lon,
        'speedKmh': speedKmh,
        'altM': altM,
        'sats': sats,
        'hdop': hdop,
        'age': age,
        'acceleration': acceleration,
      };

  factory GPSPoint.fromJson(Map<String, dynamic> json) {
    return GPSPoint(
      timestamp: json['timestamp'] as int,
      lat: (json['lat'] as num).toDouble(),
      lon: (json['lon'] as num).toDouble(),
      speedKmh: (json['speedKmh'] as num).toDouble(),
      altM: (json['altM'] as num).toDouble(),
      sats: json['sats'] as int,
      hdop: (json['hdop'] as num).toDouble(),
      age: json['age'] as int,
      acceleration: (json['acceleration'] as num).toDouble(),
    );
  }

}
