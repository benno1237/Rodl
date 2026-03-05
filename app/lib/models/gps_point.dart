import 'dart:typed_data';

class GPSPoint {
  final int timestamp;
  final double lat;
  final double lon;
  final double speedKmh;
  final double altM;
  final int sats;
  final double hdop;
  final int age;
  final double accelX;
  final double accelY;
  final double accelZ;
  final int rideIndex;

  GPSPoint({
    required this.timestamp,
    required this.lat,
    required this.lon,
    required this.speedKmh,
    required this.altM,
    required this.sats,
    required this.hdop,
    required this.age,
    this.accelX = 0.0,
    this.accelY = 0.0,
    this.accelZ = 0.0,
    this.rideIndex = 0,
  });

  double get acceleration {
    return (accelX * accelX + accelY * accelY + accelZ * accelZ);
  }

  GPSPoint copyWith({
    int? timestamp,
    double? lat,
    double? lon,
    double? speedKmh,
    double? altM,
    int? sats,
    double? hdop,
    int? age,
    double? accelX,
    double? accelY,
    double? accelZ,
    int? rideIndex,
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
      accelX: accelX ?? this.accelX,
      accelY: accelY ?? this.accelY,
      accelZ: accelZ ?? this.accelZ,
      rideIndex: rideIndex ?? this.rideIndex,
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
        'accelX': accelX,
        'accelY': accelY,
        'accelZ': accelZ,
        'rideIndex': rideIndex,
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
      accelX: json.containsKey('accelX') ? (json['accelX'] as num).toDouble() : 0.0,
      accelY: json.containsKey('accelY') ? (json['accelY'] as num).toDouble() : 0.0,
      accelZ: json.containsKey('accelZ') ? (json['accelZ'] as num).toDouble() : 0.0,
      rideIndex: json.containsKey('rideIndex') ? json['rideIndex'] as int : 0,
    );
  }

  factory GPSPoint.fromBytes(Uint8List data) {
    final byteData = ByteData.view(data.buffer);
    final rideIndex = data[0];
    final timestamp = (data[1] << 24) | (data[2] << 16) | (data[3] << 8) | data[4];
    final lat = byteData.getFloat64(5, Endian.little);
    final lon = byteData.getFloat64(13, Endian.little);
    final speedKmh = byteData.getFloat32(21, Endian.little);
    final altM = byteData.getFloat32(25, Endian.little);
    final sats = data[29];
    final hdop = byteData.getFloat32(30, Endian.little);
    final age = (data[34] << 8) | data[35];
    final accelX = byteData.getFloat32(36, Endian.little);
    final accelY = byteData.getFloat32(40, Endian.little);
    final accelZ = byteData.getFloat32(44, Endian.little);

    return GPSPoint(
      timestamp: timestamp,
      lat: lat,
      lon: lon,
      speedKmh: speedKmh,
      altM: altM,
      sats: sats,
      hdop: hdop,
      age: age,
      accelX: accelX,
      accelY: accelY,
      accelZ: accelZ,
      rideIndex: rideIndex,
    );
  }
}
