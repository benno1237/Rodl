import 'package:flutter/material.dart';
import '../models/sled.dart';

class SledCard extends StatelessWidget {
  final Sled sled;
  final VoidCallback onTap;

  const SledCard({
    super.key,
    required this.sled,
    required this.onTap,
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

              const Padding(
                padding: EdgeInsets.only(right: 16),
                child: Icon(Icons.arrow_forward),
              ),
            ],
          ),
        ),
      ),
    );
  }
}