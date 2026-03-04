 
class TrackSection {
  final String id;
  final String? name;
  // Flexible selectors: prefer one of these to identify a segment inside GPX
  final int? startIndex;
  final int? endIndex;
  final String? startTimeIso;
  final String? endTimeIso;

  TrackSection({
    required this.id,
    this.name,
    this.startIndex,
    this.endIndex,
    this.startTimeIso,
    this.endTimeIso,
  });

  factory TrackSection.fromJson(Map<String, dynamic> json) => TrackSection(
        id: json['id'] as String,
        name: json['name'] as String?,
        startIndex: json['startIndex'] is int ? json['startIndex'] as int : (json['startIndex'] is num ? (json['startIndex'] as num).toInt() : null),
        endIndex: json['endIndex'] is int ? json['endIndex'] as int : (json['endIndex'] is num ? (json['endIndex'] as num).toInt() : null),
        startTimeIso: json['startTimeIso'] as String?,
        endTimeIso: json['endTimeIso'] as String?,
      );

  Map<String, dynamic> toJson() => {
        'id': id,
        if (name != null) 'name': name,
        if (startIndex != null) 'startIndex': startIndex,
        if (endIndex != null) 'endIndex': endIndex,
        if (startTimeIso != null) 'startTimeIso': startTimeIso,
        if (endTimeIso != null) 'endTimeIso': endTimeIso,
      };
}

class Track {
  final String id;
  final String name;
  final String gpxPath; // e.g. assets/tracks/berger_alm.gpx or a remote URL
  final String? description;
  final List<TrackSection> sections;

  Track({
    required this.id,
    required this.name,
    required this.gpxPath,
    this.description,
    this.sections = const [],
  });

  factory Track.fromJson(Map<String, dynamic> json) {
    final sec = (json['sections'] as List<dynamic>?)
            ?.map((e) => TrackSection.fromJson(Map<String, dynamic>.from(e as Map)))
            .toList() ??
        <TrackSection>[];

    return Track(
      id: json['id'] as String,
      name: json['name'] as String,
      gpxPath: json['gpxPath'] as String,
      description: json['description'] as String?,
      sections: sec,
    );
  }

  Map<String, dynamic> toJson() => {
        'id': id,
        'name': name,
        'gpxPath': gpxPath,
        if (description != null) 'description': description,
        if (sections.isNotEmpty) 'sections': sections.map((s) => s.toJson()).toList(),
      };
}
