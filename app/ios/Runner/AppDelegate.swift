import Flutter
import UIKit
import UserNotifications

@main
@objc class AppDelegate: FlutterAppDelegate, FlutterImplicitEngineDelegate, UNUserNotificationCenterDelegate {
  override func application(
    _ application: UIApplication,
    didFinishLaunchingWithOptions launchOptions: [UIApplication.LaunchOptionsKey: Any]?
  ) -> Bool {
    // Request notification permission
    let center = UNUserNotificationCenter.current()
    center.delegate = self
    center.requestAuthorization(options: [.alert, .sound, .badge]) { granted, error in
      // ignore result; permissions are handled at runtime by the OS
    }

    // Set up MethodChannel to allow Dart to request start/stop of iOS local notification
    if let controller = window?.rootViewController as? FlutterViewController {
      let channel = FlutterMethodChannel(name: "rodl/foreground_service", binaryMessenger: controller.binaryMessenger)
      channel.setMethodCallHandler { call, result in
        switch call.method {
        case "start":
          let args = call.arguments as? [String: Any]
          let title = args?["title"] as? String ?? "Rodl recording"
          let text = args?["text"] as? String ?? "Recording ride"
          self.scheduleRecordingNotification(title: title, body: text)
          result(nil)
        case "stop":
          UNUserNotificationCenter.current().removePendingNotificationRequests(withIdentifiers: ["rodl_recording"])
          UNUserNotificationCenter.current().removeDeliveredNotifications(withIdentifiers: ["rodl_recording"])
          result(nil)
        default:
          result(FlutterMethodNotImplemented)
        }
      }
    }

    return super.application(application, didFinishLaunchingWithOptions: launchOptions)
  }

  func didInitializeImplicitFlutterEngine(_ engineBridge: FlutterImplicitEngineBridge) {
    GeneratedPluginRegistrant.register(with: engineBridge.pluginRegistry)
  }

  private func scheduleRecordingNotification(title: String, body: String) {
    let content = UNMutableNotificationContent()
    content.title = title
    content.body = body
    content.sound = .default
    content.userInfo = ["rodl_recording": true]

    // Trigger immediately so notification appears; tapping notification opens the app
    let trigger = UNTimeIntervalNotificationTrigger(timeInterval: 1, repeats: false)
    let request = UNNotificationRequest(identifier: "rodl_recording", content: content, trigger: trigger)
    UNUserNotificationCenter.current().add(request) { error in
      if let err = error {
        NSLog("Failed to schedule recording notification: \(err)")
      }
    }
  }

  // Ensure tapping delivered notification opens the app normally (default behavior). Keep delegate methods available for future actions.
  func userNotificationCenter(_ center: UNUserNotificationCenter, didReceive response: UNNotificationResponse, withCompletionHandler completionHandler: @escaping () -> Void) {
    completionHandler()
  }
}
