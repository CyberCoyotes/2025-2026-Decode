# FTC Team 11940 - Driver & Operator Control Guide

**Reed City Middle School | 2025-2026 Season**

---

## 🎮 GAMEPAD 1 - DRIVER CONTROLS

### **Driving**
| Control | Function |
|---------|----------|
| **Left Stick** | Strafe (side-to-side and forward/backward movement) |
| **Right Stick** | Rotate (turn left/right) |
| **Left Trigger** | Slow Motion Mode (30% speed while held) |
| **Right Trigger** | Turbo Mode (100% speed while held) |

### **Intake System**
| Control | Function |
|---------|----------|
| **Left Bumper** | Run ONLY index motors (immediate stop when released, no delay) |
| **Left Bumper + A** | Reverse ONLY index motors (no intake wheels) |
| **Right Bumper** | Run intake wheels IN + extends slides + runs BOTH index motors (sensor auto-stops top motor) |
| **Right Bumper + A** | Eject wheels OUT (reverse intake AND reverse index motors) |

> **Note:** Slides automatically extend when intake runs and retract when stopped. Wheels continue running for 600ms after slides retract. Both index motors continue for 1500ms after release to complete artifact transfer. The REV v3 color sensor can stop the top index motor early when an artifact is detected (< 3cm), while the bottom motor continues to hold it in position.

### **Drive Modes**
| Control | Function |
|---------|----------|
| **BACK Button** | Switch to Field-Centric mode |
| **START Button** | Switch to Robot-Centric mode |
| **MODE Button** | Reset Heading (only works in Field-Centric mode) |
| **A Button (alone)** | Emergency Stop (hold to freeze all drive motors) |

### **Configuration**
| Control | Function |
|---------|----------|
| **D-Pad Up** | Increase drive speed |
| **D-Pad Down** | Decrease drive speed |
| **D-Pad Right** | Increase steering sensitivity |
| **D-Pad Left** | Decrease steering sensitivity |

---

## 🎮 GAMEPAD 2 - OPERATOR CONTROLS

### **Shooting System**
| Control | Function |
|---------|----------|
| **Right Bumper** | Shoot at current preset (spins flywheel → waits for target RPM → runs index motor) |
| **Right Bumper + A** | **Reverse flywheel & index** (runs both in reverse at -1000 RPM for clearing jams) |
| **Left Bumper** | Manual index motor control forward (immediate stop, no delay) |
| **Left Bumper + A** | **Reverse index only** (runs index motors in reverse, no flywheel) |

### **Shooting Presets**
| Control | Function | Description |
|---------|----------|-------------|
| **X Button** | SHORT_RANGE Preset | Sets flywheel RPM and hood angle for close shots |
| **Y Button** | MEDIUM_RANGE Preset | Sets flywheel RPM and hood angle for mid-range shots |
| **B Button** | LONG_RANGE Preset | Sets flywheel RPM and hood angle for far shots |

### **Manual Fine-Tuning**
| Control | Function |
|---------|----------|
| **D-Pad Up** | Increase shooter RPM by 100 |
| **D-Pad Down** | Decrease shooter RPM by 100 |
| **D-Pad Right** | Hood servo UP (increase angle) |
| **D-Pad Left** | Hood servo DOWN (decrease angle) |

---

## 📊 TELEMETRY INFORMATION

### **During Match - Key Status Indicators**

**Drive Subsystem:**
- Current drive mode (Field-Centric, Robot-Centric, Turbo, Precision)
- Effective speed multiplier
- Robot heading (degrees)
- Position (X, Y coordinates in inches)

**Intake Subsystem:**
- Wheel state (INTAKE, EJECT, IDLE)
- Slide state (IN, OUT)
- Auto slide control status

**Index Subsystem:**
- Index motor state (FORWARD, REVERSE, IDLE)
- Bottom and top motor power (separate readings)
- **Sensor distance** (cm reading from REV v3 color sensor)
- **Artifact detected** (✓ YES when distance < 3cm)
- Flywheel override status (ACTIVE when shooting)
- Manual indexing active (ACTIVE/IDLE)
- Delayed stop pending (PENDING with countdown timer)

**Shooter Subsystem:**
- Current preset (SHORT_RANGE, MEDIUM_RANGE, LONG_RANGE)
- Target RPM vs Current RPM
- At target indicator (✓ YES / ✗ NO)
- Hood position
- Sequential mode status (RUNNING/IDLE)

---

## 💡 OPERATION TIPS

### **For Drivers (Gamepad 1)**

1. **Drive Mode Selection** - Use **BACK** for Field-Centric mode (move relative to field) or **START** for Robot-Centric mode (move relative to robot orientation).

2. **Reset Heading Often** - Press **MODE** button after aligning with field to reset your reference point in Field-Centric mode.

3. **Speed Control** - Use triggers for instant speed changes:
   - **Left Trigger** = Slow motion (30% speed) for precise positioning
   - **Right Trigger** = Turbo (100% speed) for quick movement
   - Release triggers for normal speed

4. **Emergency Stop** - Hold **A** button alone to immediately stop all drive motors if you lose control.

5. **Index Only Control** - Press **Left Bumper** to run ONLY index motors without intake wheels:
   - Immediate stop when released (no delay)
   - Useful for feeding artifacts to shooter
   - Press **Left Bumper + A** to reverse index motors only

6. **Intake Combo** - When you press **Right Bumper**, the system automatically:
   - Spins intake wheels IN
   - Extends slides automatically
   - Runs BOTH index motors forward
   - **Color sensor can stop top motor early when artifact detected (< 3cm)**
   - Bottom motor continues running to hold artifact
   - After release, intake wheels continue for 600ms
   - After release, index motors continue for 1500ms to complete transfer

7. **Eject Combo** - Press **Right Bumper + A** together to reverse intake AND reverse index motors (for clearing jams or ejecting artifacts from the entire system).

8. **Speed Adjustment** - Use **D-Pad Up/Down** during match to adjust base speed on the fly.

### **For Operators (Gamepad 2)**

1. **Use Presets First** - Press **X**, **Y**, or **B** to quickly select shooting distance preset (sets both RPM and hood angle).

2. **Sequential Shooting** - Hold **Right Bumper** to shoot:
   - Flywheel spins up first
   - Index motor automatically starts when flywheel reaches target RPM
   - Watch telemetry for "At Target: ✓ YES" indicator

3. **Manual Indexing** - Use **Left Bumper** for immediate index motor control without delay. Useful for:
   - Loading artifacts into shooter
   - Testing mechanisms
   - Fine positioning

4. **Reverse Index Only** - Press **Left Bumper + A** together to:
   - Run ONLY index motors in REVERSE (no flywheel)
   - Useful for clearing index jams without disturbing flywheel
   - Good for backing out artifacts that got stuck in index

5. **Clearing Flywheel Jams** - Press **Right Bumper + A** together to:
   - Run flywheel in REVERSE at -1000 RPM
   - Run both index motors in REVERSE
   - Effectively clear jams from entire shooting system
   - Release to stop both systems

6. **Fine-Tuning Shots** - After selecting a preset, use **D-Pad** to adjust:
   - **Up/Down** for RPM (±100 per press)
   - **Left/Right** for hood angle

7. **Watch the Telemetry** - Key indicators for shooting:
   - "At Target: ✓ YES" = Safe to shoot
   - "Current RPM" should match "Target RPM"
   - "Sequential Mode: RUNNING" = Shooting sequence active
   - **"Artifact Detected: ✓ YES"** = Artifact ready in index (from sensor)

---

## ⚠️ IMPORTANT NOTES

### **REV v3 Color Sensor (Smart Artifact Detection)**
- **Distance Threshold**: 3.0 cm
- **Function**: Automatically stops top index motor when artifact is detected during intake
- Bottom motor continues to hold artifact securely in position
- **Flywheel Override**: Sensor is disabled when shooting (flywheel running) to allow artifact to feed through
- Monitor "Sensor Distance" and "Artifact Detected" in telemetry for real-time feedback

### **Intake & Index Timing**
- **Gamepad 1 Left Bumper**: Index motors only - immediate stop when released (no delay)
- **Gamepad 1 Left Bumper + A**: Reverse index motors only (no intake wheels)
- **Gamepad 1 Right Bumper**: Intake wheels stop after 600ms delay, index motors stop after 1500ms delay
- **Gamepad 1 Right Bumper + A**: Eject combo - reverses intake AND index motors (no delay)
- **Gamepad 2 Left Bumper**: Immediate stop when released (no delay)
- **Gamepad 2 Left Bumper + A**: Reverse index only combo - runs index motors in reverse
- **Gamepad 2 Right Bumper + A**: Reverse combo - runs flywheel and index in reverse for jam clearing
- Manual indexing cancels any pending delayed stops from intake combo

### **Drive Mode Differences**
- **Field-Centric**: Robot moves relative to field orientation (forward is always away from drivers)
- **Robot-Centric**: Robot moves relative to robot orientation (forward is robot's front)
- **Slow Motion** (Left Trigger): 30% speed for precise positioning
- **Turbo** (Right Trigger): 100% speed for quick movement

### **Shooter Safety**
- Flywheel must reach target RPM before index motor starts (automatic)
- Always verify "At Target: ✓ YES" in telemetry before shooting
- Color sensor automatically detects artifact presence before shooting
- Sensor is overridden during shooting to allow artifact to feed through
- Use presets during competition for consistent, tested shots
- Manual RPM adjustment is for testing and fine-tuning only
- **Reverse function (RB + A)** is for jam clearing only - not for normal operation

---

## 🔧 DEFAULT SETTINGS

| Setting | Value |
|---------|-------|
| Base Speed | 0.85 (85%) |
| Slow Motion Speed | 0.30 (30%) |
| Turbo Speed | 1.0 (100%) |
| Sensitivity | 1.2 |
| Deadzone | 0.1 |
| Index Bottom Motor Delay | 1500ms |
| Index Top Motor Delay | 1500ms (sensor can override) |
| Intake Wheel Stop Delay | 600ms |
| Shooter RPM Increment | 100 RPM per press |
| Hood Position Increment | 0.05 per press |
| **Color Sensor Distance Threshold** | **3.0 cm** |
| **Reverse Flywheel RPM** | **-1000 RPM** |

---

## 📞 SUPPORT

**OpMode Name:** Team 11940 TeleOp

**Source File:** `PrimeTeleOp.java`

**Hardware:** REV v3 Color Sensor for artifact detection

**Last Updated:** 2025-12-03

---

*Print this guide and keep it at the driver station for quick reference during matches!*
