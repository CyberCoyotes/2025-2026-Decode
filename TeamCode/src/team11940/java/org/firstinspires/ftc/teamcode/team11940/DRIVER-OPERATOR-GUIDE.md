# FTC Team 11940 - Driver & Operator Control Guide

**Reed City Middle School | 2025-2026 Season**

---

## 🎮 GAMEPAD 1 - DRIVER CONTROLS

### **Driving**
| Control | Function |
|---------|----------|
| **Left Stick** | Strafe (side-to-side and forward/backward movement) |
| **Right Stick** | Rotate (turn left/right) |

### **Intake System**
| Control | Function |
|---------|----------|
| **Right Bumper** | Run intake wheels IN + extends slides automatically + runs bottom index motor |
| **Left Bumper** | Eject wheels OUT (slides retract automatically) |

> **Note:** Slides automatically extend when intake runs and retract when stopped. Wheels continue running for 300ms after slides retract to complete artifact transfer.

### **Drive Modes**
| Control | Function |
|---------|----------|
| **X Button** | Toggle between Field-Centric and Robot-Centric modes |
| **B Button** | Toggle Turbo Mode (maximum speed) |
| **A Button** | Emergency Stop (hold to freeze all drive motors) |

### **Configuration**
| Control | Function |
|---------|----------|
| **Options Button** | Reset Heading (only works in Field-Centric mode) |
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
| **Left Bumper** | Manual index motor control (immediate stop, no delay) |

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
- Manual indexing active (ACTIVE/IDLE)
- Delayed stop pending (PENDING/NONE)

**Shooter Subsystem:**
- Current preset (SHORT_RANGE, MEDIUM_RANGE, LONG_RANGE)
- Target RPM vs Current RPM
- At target indicator (✓ YES / ✗ NO)
- Hood position
- Sequential mode status (RUNNING/IDLE)

---

## 💡 OPERATION TIPS

### **For Drivers (Gamepad 1)**

1. **Start in Field-Centric Mode** - The robot will move relative to the field regardless of robot orientation. Press **X** to switch to Robot-Centric if needed.

2. **Reset Heading Often** - Press **Options** button after aligning with field to reset your reference point in Field-Centric mode.

3. **Emergency Stop** - Hold **A** button to immediately stop all drive motors if you lose control.

4. **Intake Combo** - When you press **Right Bumper**, three things happen:
   - Intake wheels spin IN
   - Slides extend automatically
   - Bottom index motor runs to transfer artifact
   - After release, motors continue for 900ms to complete transfer

5. **Speed Adjustment** - Use **D-Pad Up/Down** during match to adjust speed on the fly.

### **For Operators (Gamepad 2)**

1. **Use Presets First** - Press **X**, **Y**, or **B** to quickly select shooting distance preset (sets both RPM and hood angle).

2. **Sequential Shooting** - Hold **Right Bumper** to shoot:
   - Flywheel spins up first
   - Index motor automatically starts when flywheel reaches target RPM
   - Watch telemetry for "At Target: ✓ YES" indicator

3. **Manual Indexing** - Use **Left Bumper** for immediate index motor control without delay. Useful for:
   - Loading artifacts into shooter
   - Clearing jams
   - Testing mechanisms

4. **Fine-Tuning Shots** - After selecting a preset, use **D-Pad** to adjust:
   - **Up/Down** for RPM (±100 per press)
   - **Left/Right** for hood angle

5. **Watch the Telemetry** - Key indicators for shooting:
   - "At Target: ✓ YES" = Safe to shoot
   - "Current RPM" should match "Target RPM"
   - "Sequential Mode: RUNNING" = Shooting sequence active

---

## ⚠️ IMPORTANT NOTES

### **Intake & Index Timing**
- **Gamepad 1 Right Bumper**: Has 900ms delay after release to complete artifact transfer
- **Gamepad 2 Left Bumper**: Immediate stop when released (no delay)
- Manual indexing cancels any pending delayed stops from intake combo

### **Drive Mode Differences**
- **Field-Centric**: Robot moves relative to field orientation (forward is always away from drivers)
- **Robot-Centric**: Robot moves relative to robot orientation (forward is robot's front)
- **Turbo**: Maximum speed override for quick repositioning
- **Precision**: (Available but not assigned) - Reduced speed for fine positioning

### **Shooter Safety**
- Flywheel must reach target RPM before index motor starts (automatic)
- Always verify "At Target: ✓ YES" in telemetry before shooting
- Use presets during competition for consistent, tested shots
- Manual RPM adjustment is for testing and fine-tuning only

---

## 🔧 DEFAULT SETTINGS

| Setting | Value |
|---------|-------|
| Base Speed | 0.85 (85%) |
| Sensitivity | 1.2 |
| Deadzone | 0.1 |
| Index Bottom Motor Delay | 900ms |
| Intake Wheel Stop Delay | 300ms |
| Shooter RPM Increment | 100 RPM per press |
| Hood Position Increment | 0.05 per press |

---

## 📞 SUPPORT

**OpMode Name:** Team 11940 TeleOp

**Source File:** `PrimeTeleOp.java`

**Last Updated:** 2025-11-25

---

*Print this guide and keep it at the driver station for quick reference during matches!*
