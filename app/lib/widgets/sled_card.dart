import 'package:flutter/material.dart';
import '../models/sled.dart';

class SledCard extends StatelessWidget {
  final Sled sled;
  final VoidCallback onTap;
  final VoidCallback? onFavoriteToggle;

  /// Not currently visible in a discovery scan - greyed out and untappable.
  final bool dimmed;

  /// Seen in the latest discovery scan.
  final bool online;

  /// Connected by someone else (board advertises in-use).
  final bool inUse;

  /// A connection attempt to this sled is running.
  final bool connecting;

  /// Name advertised by the PCB (falls back to [Sled.name]).
  final String? displayName;

  const SledCard({
    super.key,
    required this.sled,
    required this.onTap,
    this.onFavoriteToggle,
    this.dimmed = false,
    this.online = false,
    this.inUse = false,
    this.connecting = false,
    this.displayName,
  });

  @override
  Widget build(BuildContext context) {
    final name = (displayName != null && displayName!.isNotEmpty)
        ? displayName!
        : sled.name;

    final Widget trailing;
    if (connecting) {
      trailing = const SizedBox(
        width: 24,
        height: 24,
        child: CircularProgressIndicator(strokeWidth: 2.5),
      );
    } else {
      trailing = IconButton(
        icon: Icon(
          sled.isFavorite ? Icons.star : Icons.star_border,
          color: sled.isFavorite
              ? Theme.of(context).colorScheme.primary
              : Theme.of(context).iconTheme.color,
        ),
        onPressed: onFavoriteToggle,
      );
    }

    return Card(
      margin: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
      child: InkWell(
        onTap: onTap,
        child: SizedBox(
          height: 100, // 👈 controls card height
          child: Row(
            children: [
              // Image section
              Opacity(
                opacity: dimmed ? 0.4 : 1.0,
                child: SizedBox(
                  height: double.infinity,
                  child: Image.asset(
                    sled.imagePath,
                    fit: BoxFit.fitHeight,
                  ),
                ),
              ),

              const SizedBox(width: 16),

              // Text section
              Expanded(
                child: Opacity(
                  opacity: dimmed ? 0.45 : 1.0,
                  child: Column(
                    mainAxisAlignment: MainAxisAlignment.center,
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Row(
                        children: [
                          Flexible(
                            child: Text(
                              name,
                              style: Theme.of(context).textTheme.titleLarge,
                              overflow: TextOverflow.ellipsis,
                            ),
                          ),
                          const SizedBox(width: 6),
                          if (online)
                            Container(
                              width: 9,
                              height: 9,
                              decoration: const BoxDecoration(
                                shape: BoxShape.circle,
                                color: Colors.green,
                              ),
                            ),
                          if (inUse)
                            const Icon(
                              Icons.lock,
                              size: 15,
                              color: Colors.orange,
                            ),
                        ],
                      ),
                      const SizedBox(height: 8),
                      Text(
                        inUse
                            ? 'In use by another user'
                            : online
                                ? 'Nearby - tap to connect'
                                : 'Not nearby',
                        style: TextStyle(
                          fontSize: 13,
                          color: inUse
                              ? Colors.orange
                              : online
                                  ? Colors.green.shade700
                                  : Colors.grey,
                        ),
                      ),
                    ],
                  ),
                ),
              ),

              trailing,
              const SizedBox(width: 4),
            ],
          ),
        ),
      ),
    );
  }
}
