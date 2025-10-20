# Limelight 3A Quick Start Guide

## For the Practice Bot

### Quick Setup (5 minutes)

1. **Connect Hardware**
   - Plug Limelight 3A into Control Hub USB port
   - Wait 10-15 seconds for boot

2. **Configure in Robot Controller**
   - Open Robot Controller app → Configuration
   - Find Limelight device (shows with IP address below)
   - Rename to: `limelight`
   - Enter Limelight IP: `192.168.1.11` (default)
   - Save configuration

3. **Configure Pipeline (Web Interface)**
   - Browser: `http://192.168.1.11:5801`
   - Pipeline Type: **AprilTag (Fiducial)**
   - Tag Family: **36h11**
   - Save pipeline as index 0

4. **Test**
   - Run OpMode: **"Test: Limelight AprilTag"**
   - Point camera at any 36h11 AprilTag
   - Check Driver Station telemetry for tag ID

### What's Included

| File | Purpose |
|------|---------|
| `LimelightSubsystem.java` | Main Limelight control class |
| `TestLimelight.java` | Standalone test OpMode |
| `RobotContainer.java` | Main TeleOp with Limelight integrated |
| `LIMELIGHT_SETUP.md` | Complete setup documentation |

### Using in Your Code

```java
// Initialize (already done in RobotContainer)
limelightSub = new LimelightSubsystem(hardwareMap);

// In your loop
limelightSub.update();

// Check for tags
if (limelightSub.getAprilTagCount() > 0) {
    // Do something when tags detected
}

// Get specific tag
if (limelightSub.isTagDetected(5)) {
    LLResultTypes.FiducialResult tag = limelightSub.getTagById(5);
    // Use tag data
}

// Cleanup
limelightSub.stop();
```

### Telemetry Display

When running **"TeleOp"** OpMode, you'll see:

```
--- LIMELIGHT ---
Status: Temp: 42.5°C, CPU: 25.3%, FPS: 90, Pipeline: 0
AprilTags: Detected 2 tag(s): ID 5 (12.3°, -5.1°), ID 7 (-8.2°, 3.4°)
Target Angle: X: 12.3°, Y: -5.1°
Latency: 15.2 ms
```

### Tag Family Info

**2025-2026 DECODE Season uses 36h11 family**

- More robust than smaller families
- Better detection at angles and distances
- Standard for all FTC DECODE game elements

### Common Issues

| Problem | Solution |
|---------|----------|
| No telemetry data | Check USB connection, wait for boot |
| Tags not detected | Verify pipeline set to 36h11 |
| Wrong tag IDs | Make sure using 36h11 printed tags |
| High latency | Check Control Hub CPU load |

### Next Steps

1. ✅ Run **TestLimelight** to verify everything works
2. Test detection at different distances
3. Test detection at different angles
4. Integrate into autonomous routines
5. Add navigation/alignment code using tag data

### Need Help?

- See `LIMELIGHT_SETUP.md` for detailed instructions
- Check Limelight docs: https://limelightvision.io/
- Sample code: `SensorLimelight3A.java` in FtcRobotController/samples

---

**Ready to test!** Run "Test: Limelight AprilTag" from Driver Station.
