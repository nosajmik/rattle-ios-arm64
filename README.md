# rattle-ios-arm64
Bastardized rattle code (works only with -t for trial) for use on jailbroken iOS Devices.

### Nosajmik Changelog
* Removed soundcard includes, which iOS doesn't support.
* All functions that depend on the soundcard headers are there, but commented out.
* Inline assembly register names changed for arm64 (`r0` -> `w0`, etc.)

### How to build this, just in case
1. Open the workspace file in Xcode. Under the Product tab in Xcode's menu bar, ensure that 'Scheme' is set to 'rattle-ios-arm64' and 'Destination' is set for your device.
1. If your device is not on the list, disconnect it from your Mac. Open Xcode first, then connect the device again.
1. I tested this on an iPhone 11 running iOS 14.6. If you are targeting a different version of iOS, go to 'Build Settings' for the 'rattle-ios-arm64' target and change 'iOS Deployment Target' to your device's version.
1. Build for release (Cmd + Shift + I). This will produce a binary at `Build/Products/Release-iphoneos`.

### How to ship rattle to an iDevice (read this first)
1. I have a binary that _should_ work on most recent arm64 iOS devices, if not all. This `rattle-ios-arm64` binary is in `Build/Products/Release-iphoneos`.
1. Copy the binary and `entitlements.plist` to your iDevice.
1. With a shell open on the iDevice, run the following to run rattle in trial mode:
```
chmod +x <path to rattle-ios-arm64 binary>
ldid -S <path to entitlements.plist> <path to rattle-ios-arm64 binary>
<path to rattle-ios-arm64 binary> -t
```
