package com.example.rodl

import android.content.Intent
import android.os.Bundle
import androidx.core.content.ContextCompat
import io.flutter.embedding.android.FlutterActivity
import io.flutter.embedding.engine.FlutterEngine
import io.flutter.plugin.common.MethodChannel

class MainActivity : FlutterActivity() {
	private val CHANNEL = "rodl/foreground_service"

	override fun configureFlutterEngine(flutterEngine: FlutterEngine) {
		super.configureFlutterEngine(flutterEngine)

		MethodChannel(flutterEngine.dartExecutor.binaryMessenger, CHANNEL).setMethodCallHandler { call, result ->
			when (call.method) {
				"start" -> {
					val args = call.arguments as? Map<*, *>
					val title = args?.get("title") as? String ?: "Rodl recording"
					val text = args?.get("text") as? String ?: "Recording ride"
					val intent = Intent(this, ForegroundLocationService::class.java).apply {
						putExtra("title", title)
						putExtra("text", text)
					}
					try {
						ContextCompat.startForegroundService(this, intent)
						result.success(null)
					} catch (e: Exception) {
						result.error("start_error", e.message, null)
					}
				}
				"stop" -> {
					val intent = Intent(this, ForegroundLocationService::class.java)
					try {
						stopService(intent)
						result.success(null)
					} catch (e: Exception) {
						result.error("stop_error", e.message, null)
					}
				}
				else -> result.notImplemented()
			}
		}
	}
}
