import 'package:flutter/material.dart';
import '../models/sled.dart';

class SledCard extends StatelessWidget {
  final Sled sled;
  final VoidCallback onTap;
  final VoidCallback? onFavoriteToggle;

  const SledCard({
    super.key,
    required this.sled,
    required this.onTap,
    this.onFavoriteToggle,
  });

  @override
  Widget build(BuildContext context) {
    return Card(
      margin: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
      child: InkWell(
        onTap: onTap,
        child: SizedBox(
          height: 100, // 👈 controls card height
          child: Row(
            children: [
              // Image section
              SizedBox(
                height: double.infinity,
                child: Image.asset(
                  sled.imagePath,
                  fit: BoxFit.fitHeight,
                ),
              ),

              const SizedBox(width: 16),

              // Text section
              Expanded(
                child: Column(
                  mainAxisAlignment: MainAxisAlignment.center,
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text(
                      sled.name,
                      style: Theme.of(context).textTheme.titleLarge,
                    ),
                    const SizedBox(height: 8),
                    Text("ID: ${sled.id}"),
                  ],
                ),
              ),

              Column(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  IconButton(
                    icon: Icon(
                      sled.isFavorite ? Icons.star : Icons.star_border,
                      color: sled.isFavorite
                          ? Theme.of(context).colorScheme.primary
                          : Theme.of(context).iconTheme.color,
                    ),
                    onPressed: onFavoriteToggle,
                  ),
                ],
              ),
            ],
          ),
        ),
      ),
    );
  }
}