plugins {
    id("com.android.application")
    id("org.jetbrains.kotlin.android")
    id("org.jetbrains.kotlin.plugin.compose") version "2.0.0"
}

android {
    namespace = "com.example.noisegenadjustedtilt"
    compileSdk = 34

    defaultConfig {
        applicationId = "com.example.noisegenadjustedtilt"
        minSdk = 31
        targetSdk = 34
        versionCode = 1
        versionName = "1.0"
    }

    buildFeatures {
        compose = true
    }
    //composeOptions {
    //    kotlinCompilerExtensionVersion = "1.5.14"
    //}

    // (optional) enable Java 17 if Android Studio orders so
    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }
    kotlinOptions {
        jvmTarget = "17"
    }
}

dependencies {
    val composeBom = platform("androidx.compose:compose-bom:2024.06.00")
    implementation(composeBom)
    androidTestImplementation(composeBom)

    implementation("androidx.activity:activity-compose:1.9.0")
    implementation("androidx.compose.ui:ui")
    implementation("androidx.compose.material3:material3:1.2.1")
    implementation("com.google.android.material:material:1.12.0")
    implementation("androidx.compose.ui:ui-tooling-preview")
    // Unit-tests (src/test)
    testImplementation("junit:junit:4.13.2")

// Instrumented tests (src/androidTest)
    androidTestImplementation("androidx.test:core:1.6.1")          // InstrumentationRegistry
    androidTestImplementation("androidx.test.ext:junit:1.2.1")     // AndroidJUnit4, @Test
    androidTestImplementation("androidx.test.espresso:espresso-core:3.6.1")
// ViewModel in Compose + SavedStateHandle support
    implementation("androidx.lifecycle:lifecycle-viewmodel-compose:2.8.4")
    implementation("androidx.lifecycle:lifecycle-viewmodel-savedstate:2.8.4")


    debugImplementation("androidx.compose.ui:ui-tooling")
}

