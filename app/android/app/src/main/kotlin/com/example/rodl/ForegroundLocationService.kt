package com.example.rodl

import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.PendingIntent
import android.app.Service
import android.content.Context
import android.content.Intent
import android.content.SharedPreferences
import android.location.Location
import android.os.Looper
import com.google.android.gms.location.FusedLocationProviderClient
import com.google.android.gms.location.LocationCallback
import com.google.android.gms.location.LocationRequest
import com.google.android.gms.location.LocationResult
import com.google.android.gms.location.LocationServices
import android.os.Build
import android.os.IBinder
import android.util.Log
import androidx.core.app.NotificationCompat
import org.json.JSONArray
import org.json.JSONObject
import java.lang.Exception

class ForegroundLocationService : Service() {
    private val TAG = "ForegroundLocationSvc"
    private val CHANNEL_ID = "rodl_location_channel"
    private val NOTIF_ID = 7812
    private lateinit var fusedClient: FusedLocationProviderClient
    private var locationCallback: LocationCallback? = null
    private lateinit var prefs: SharedPreferences
    private val PREF_KEY = "bg_recording_points_v1"

    override fun onCreate() {
        super.onCreate()
        // Use Flutter's default SharedPreferences file so Dart can read the same values.
        prefs = getSharedPreferences("FlutterSharedPreferences", Context.MODE_PRIVATE)
        fusedClient = LocationServices.getFusedLocationProviderClient(this)
    }

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        val title = intent?.getStringExtra("title") ?: "Rodl recording"
        val text = intent?.getStringExtra("text") ?: "Recording ride"

        createNotificationChannel()

        val stopIntent = Intent(this, ForegroundLocationService::class.java).apply {
            action = ACTION_STOP
        }
        val stopPending = PendingIntent.getService(
            this, 0, stopIntent, pendingIntentFlags()
        )

        val launchIntent = packageManager.getLaunchIntentForPackage(packageName) ?: Intent(this, MainActivity::class.java)
        launchIntent.addFlags(Intent.FLAG_ACTIVITY_NEW_TASK or Intent.FLAG_ACTIVITY_CLEAR_TOP or Intent.FLAG_ACTIVITY_SINGLE_TOP)
        val launchPending = PendingIntent.getActivity(
            this, 0, launchIntent, pendingIntentFlags()
        )

        val notification: Notification = NotificationCompat.Builder(this, CHANNEL_ID)
            .setContentTitle(title)
            .setContentText(text)
            .setContentIntent(launchPending)
            .setSmallIcon(R.mipmap.ic_launcher)
            .addAction(android.R.drawable.ic_media_pause, "Stop", stopPending)
            .setOngoing(true)
            .build()

        startForeground(NOTIF_ID, notification)

        if (intent?.action == ACTION_STOP) {
            stopSelf()
            return START_NOT_STICKY
        }

        startLocationUpdates()
        return START_STICKY
    }

    private fun pendingIntentFlags(): Int {
        return if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            PendingIntent.FLAG_UPDATE_CURRENT or PendingIntent.FLAG_IMMUTABLE
        } else {
            PendingIntent.FLAG_UPDATE_CURRENT
        }
    }

    private fun createNotificationChannel() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            val chan = NotificationChannel(CHANNEL_ID, "Rodl Location", NotificationManager.IMPORTANCE_LOW)
            val manager = getSystemService(Context.NOTIFICATION_SERVICE) as NotificationManager
            manager.createNotificationChannel(chan)
        }
    }

    private fun startLocationUpdates() {
        try {
            if (locationCallback != null) return

            val request = LocationRequest.Builder(1000L)
                .setPriority(LocationRequest.PRIORITY_HIGH_ACCURACY)
                .setMinUpdateIntervalMillis(500L)
                .build()

            locationCallback = object : LocationCallback() {
                override fun onLocationResult(result: LocationResult) {
                    val loc = result.lastLocation
                    if (loc != null) {
                        handleLocation(loc)
                    }
                }
            }

            fusedClient.requestLocationUpdates(request, locationCallback as LocationCallback, Looper.getMainLooper())
        } catch (e: SecurityException) {
            Log.e(TAG, "Missing location permission: ${e.message}")
        } catch (e: Exception) {
            Log.e(TAG, "Failed to request location updates: ${e.message}")
        }
    }

    private fun handleLocation(location: Location) {
        try {
            val raw = prefs.getString(PREF_KEY, "[]") ?: "[]"
            val arr = JSONArray(raw)
            val obj = JSONObject()
            obj.put("timestamp", location.time)
            obj.put("lat", location.latitude)
            obj.put("lon", location.longitude)
            obj.put("speedKmh", location.speed * 3.6)
            obj.put("altM", location.altitude)
            obj.put("sats", 0)
            obj.put("hdop", location.accuracy)
            obj.put("age", 0)
            arr.put(obj)
            prefs.edit().putString(PREF_KEY, arr.toString()).apply()
        } catch (e: Exception) {
            Log.e(TAG, "Failed to persist point: ${e.message}")
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        locationCallback?.let { fusedClient.removeLocationUpdates(it) }
        locationCallback = null
    }

    override fun onBind(intent: Intent?): IBinder? {
        return null
    }

    companion object {
        const val ACTION_STOP = "com.example.rodl.ACTION_STOP_FOREGROUND"
    }
}
