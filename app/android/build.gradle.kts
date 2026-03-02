import org.jetbrains.kotlin.gradle.tasks.KotlinCompile
import org.gradle.jvm.toolchain.JavaLanguageVersion

allprojects {
    repositories {
        google()
        mavenCentral()
    }
}

val newBuildDir: Directory =
    rootProject.layout.buildDirectory
        .dir("../../build")
        .get()
rootProject.layout.buildDirectory.value(newBuildDir)

subprojects {
    val newSubprojectBuildDir: Directory = newBuildDir.dir(project.name)
    project.layout.buildDirectory.value(newSubprojectBuildDir)
}
subprojects {
    project.evaluationDependsOn(":app")
}

// Ensure all Java/Kotlin compilation targets use Java 17 to avoid JDK warnings
subprojects {
    // For any JavaCompile tasks, set source/target and add a safe compiler arg
    tasks.withType(org.gradle.api.tasks.compile.JavaCompile::class.java).configureEach {
        sourceCompatibility = "17"
        targetCompatibility = "17"
        options.compilerArgs.add("-Xlint:-options")
    }

    // Configure Java toolchain where the Java plugin is applied
    plugins.withId("java") {
        extensions.findByType(org.gradle.api.plugins.JavaPluginExtension::class.java)
            ?.toolchain?.languageVersion?.set(JavaLanguageVersion.of(17))
    }

    // Note: Kotlin compilation is configured at module level; avoid using deprecated kotlinOptions here.
}

tasks.register<Delete>("clean") {
    delete(rootProject.layout.buildDirectory)
}
