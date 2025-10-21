Best Practices for Drive Motor Ports:
Priority 1: Use Ports 0-3 for Critical Motors

Ports 0-3 are on the primary I²C bus and have faster communication than ports on the expansion hub
These should be your drive motors since they need the fastest response time

Priority 2: Distribute Load Across Both Buses
For a mecanum drive, a good configuration is:
Control Hub (Ports 0-3):

Port 0: leftFront
Port 1: leftBack
Port 2: rightFront
Port 3: rightBack

This keeps all drive motors on the faster primary bus.
Priority 3: Balance Current Draw

REV hubs can handle 10A continuous per port (15A peak)
Total hub limit is 15A continuous across all ports
Avoid putting multiple high-current motors on adjacent ports when possible
For drive motors, this usually isn't an issue, but keep heavy mechanisms (like lifts) spread out

Priority 4: Encoder Ports (Important!)
From the Road Runner documentation you provided:

"Note that, for technical reasons, ports 0 and 3 on REV hubs are more accurate at high speeds and should be used for the parallel dead wheels."

If you're using dead wheels (odometry pods):

Ports 0 & 3: Parallel dead wheels (most accurate)
Port 1 or 2: Perpendicular dead wheel

If you're using drive encoders for odometry, prioritize:

Ports 0 & 3: Left and right side motors (better encoder accuracy)

Typical FTC Mecanum Setup:
Control Hub:
Port 0: leftFront   (drive + encoder if using drive encoders)
Port 1: leftBack    (drive)
Port 2: rightBack   (drive)
Port 3: rightFront  (drive + encoder if using drive encoders)
```

**Expansion Hub (if needed):**
```
Port 0: intake motor
Port 1: lift motor
Port 2: arm motor
Port 3: (spare or another mechanism)
Why This Matters:

Latency: Ports 0-3 have ~10ms faster response than expansion hub ports
Encoder Accuracy: Ports 0 & 3 have better high-speed encoder reading
Bulk Reads: Control Hub can read all motors in one operation (faster than reading individually)

Additional Tips:

Use bulk caching (already in your MecanumDrive.java):

```
java  module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
```

Keep wiring neat: Shorter wires = less resistance = better performance
Label everything
