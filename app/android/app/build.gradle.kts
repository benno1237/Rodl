plugins {
    id("com.android.application")
    id("kotlin-android")
    // The Flutter Gradle Plugin must be applied after the Android and Kotlin Gradle plugins.
    id("dev.flutter.flutter-gradle-plugin")
}

// Pre-build hook to zip tiles directory using a Gradle Zip task (cross-platform)
import org.gradle.api.tasks.bundling.Zip

val zipTiles = tasks.register<Zip>("zipTiles") {
    // compute the Flutter project root (android/app -> ../..)
    val projectRoot = project.projectDir.parentFile.parentFile
    val tilesDir = File(projectRoot, "assets/tiles")
    val imagesDir = File(projectRoot, "assets/images")

    // Only run when a tiles folder exists
    onlyIf {
        tilesDir.exists() && tilesDir.isDirectory
    }

    doFirst {
        println("Zipping assets/tiles from $tilesDir into $imagesDir/tiles.zip")
        imagesDir.mkdirs()
    }

    // Put entries under assets/tiles/... inside the archive
    from(tilesDir) {
        into("assets/tiles")
    }

    archiveFileName.set("tiles.zip")
    destinationDirectory.set(imagesDir)
}

tasks.named("preBuild") {
    dependsOn(zipTiles)
}

// Ensure the Flutter compile task explicitly depends on our zip task so Gradle
// doesn't complain about an implicit dependency on the produced tiles.zip.
tasks.matching { it.name == "compileFlutterBuildRelease" }.configureEach {
    dependsOn(zipTiles)
}

android {
    namespace = "com.example.rodl"
    compileSdk = flutter.compileSdkVersion
    ndkVersion = flutter.ndkVersion

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }

    kotlinOptions {
        jvmTarget = JavaVersion.VERSION_17.toString()
    }

    defaultConfig {
        // TODO: Specify your own unique Application ID (https://developer.android.com/studio/build/application-id.html).
        applicationId = "com.example.rodl"
        // You can update the following values to match your application needs.
        // For more information, see: https://flutter.dev/to/review-gradle-config.
        minSdk = flutter.minSdkVersion
        targetSdk = flutter.targetSdkVersion
        versionCode = flutter.versionCode
        versionName = flutter.versionName
    }

    buildTypes {
        release {
            // TODO: Add your own signing config for the release build.
            // Signing with the debug keys for now, so `flutter run --release` works.
            signingConfig = signingConfigs.getByName("debug")
        }
    }
}

flutter {
    source = "../.."
}

dependencies {
    implementation("com.google.android.gms:play-services-location:21.0.1")
}
