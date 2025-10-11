# FTC Java Naming Conventions Quick Reference

## Packages (Folders)
**Format:** `alllowercase` (no underscores, no capitals)

```
✅ practicebot
✅ team11940
✅ common
✅ subsystems
✅ autonomous

❌ practiceBot (capitals)
❌ practice_bot (underscores)
```

---

## Classes
**Format:** `PascalCase` (every word starts with capital)

### OpModes
```
✅ PracticeBotTeleOp.java
✅ Team11940AutoBlue.java
✅ TestLimelight.java

❌ practiceBotTeleOp.java
❌ Practice_Bot_TeleOp.java
```

### Subsystems
```
✅ DriveSubsystem.java
✅ ShooterSubsystem.java
✅ LimelightSubsystem.java

❌ driveSubsystem.java
❌ Drive_Subsystem.java
```

### Configurations
```
✅ RobotConfig.java
✅ PracticeRobotConfig.java
✅ HardwareConstants.java

❌ robot_config.java
```

---

## Variables
**Format:** `camelCase` (first word lowercase, rest capitalized)

```java
✅ private DcMotor leftFrontMotor;
✅ private double targetPower;
✅ private boolean isClawOpen;
✅ private int currentPosition;

❌ private DcMotor LeftFrontMotor;
❌ private DcMotor left_front_motor;
❌ private DcMotor lf;  // too short
```

### Subsystem Instances
```java
✅ private DriveSubsystem driveSubsystem;
✅ private ShooterSubsystem shooterSub;
✅ private LimelightSubsystem limelightSub;

❌ private DriveSubsystem DRIVE;
❌ private DriveSubsystem d;
```

---

## Methods
**Format:** `camelCase` starting with a **verb**

```java
✅ public void initialize()
✅ public void setPower(double power)
✅ public double getTargetPower()
✅ public boolean isFinished()
✅ public void runShooter()
✅ public void stopShooter()

❌ public void Initialize()  // wrong case
❌ public void set_power()   // underscores
❌ public void Power()       // not a verb
```

### Common Method Patterns
- **Setters:** `setPower()`, `setPosition()`, `setDirection()`
- **Getters:** `getPower()`, `getPosition()`, `getCurrentState()`
- **Boolean:** `isFinished()`, `isRunning()`, `hasTarget()`, `canMove()`
- **Actions:** `start()`, `stop()`, `run()`, `update()`, `reset()`

---

## Constants
**Format:** `ALL_CAPS_WITH_UNDERSCORES`

```java
✅ public static final double WHEEL_DIAMETER = 4.0;
✅ public static final double TRACK_WIDTH = 15.5;
✅ private static final int MAX_RPM = 5000;
✅ private static final String LIMELIGHT_NAME = "limelight";
✅ private static final double DEADZONE_THRESHOLD = 0.1;

❌ public static final double wheelDiameter = 4.0;
❌ public static final double wheel_diameter = 4.0;
❌ public static final double WD = 4.0;  // too short
```

---

## Hardware Map Names
**Format:** `lowercase_with_underscores` (matches Robot Configuration app)

```java
✅ hardwareMap.get(DcMotor.class, "left_front_motor");
✅ hardwareMap.get(DcMotor.class, "shooter_motor_1");
✅ hardwareMap.get(Servo.class, "claw_servo");
✅ hardwareMap.get(Limelight3A.class, "limelight");

❌ hardwareMap.get(DcMotor.class, "leftFrontMotor");  // use snake_case
❌ hardwareMap.get(DcMotor.class, "LeftFrontMotor");
❌ hardwareMap.get(DcMotor.class, "lf");  // too short
```

---

## OpMode Annotations
**Format:** Descriptive names with proper capitalization

```java
✅ @TeleOp(name = "Practice Bot TeleOp", group = "Competition")
✅ @TeleOp(name = "Team 11940 TeleOp", group = "Team11940")
✅ @Autonomous(name = "Blue Alliance Auto", group = "Competition")
✅ @TeleOp(name = "Test: Limelight", group = "Test")

❌ @TeleOp(name = "teleop", group = "a")  // not descriptive
❌ @TeleOp(name = "TELEOP", group = "MAIN")  // all caps
```

---

## Comments & Documentation
**Format:** JavaDoc style for public methods

```java
/**
 * Set the power for all drive motors
 * 
 * @param power Motor power from -1.0 to 1.0
 */
public void setPower(double power) {
    // Implementation here
}

// Single-line comments for internal logic
// Use double-slash for brief explanations
```

---

## File Organization Example

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
├── practicebot/                    // Package: all lowercase
│   ├── PracticeBotTeleOp.java     // OpMode: PascalCase
│   ├── PracticeBotAutoBlue.java
│   ├── TestLimelight.java
│   ├── ShooterSubsystem.java      // Subsystem: PascalCase
│   ├── LimelightSubsystem.java
│   └── PracticeRobotConfig.java   // Config: PascalCase
├── team11940/
│   ├── Team11940TeleOp.java
│   └── Robot11940Config.java
└── common/
    ├── subsystems/
    │   └── DriveSubsystem.java
    └── util/
        └── MathUtil.java
```

---

## Complete Class Template

```java
package org.firstinspires.ftc.teamcode.practicebot;  // Package: lowercase

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * JavaDoc comment for class documentation
 */
@TeleOp(name = "Example TeleOp", group = "Examples")
public class ExampleTeleOp {  // Class: PascalCase
    
    // Constants: ALL_CAPS_WITH_UNDERSCORES
    private static final double MAX_POWER = 1.0;
    private static final int COUNTS_PER_REV = 1440;
    
    // Instance variables: camelCase
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private double targetPower;
    private boolean isRunning;
    
    // Subsystems: camelCase
    private DriveSubsystem driveSubsystem;
    
    /**
     * JavaDoc for public methods
     * @param power Motor power value
     */
    public void setPower(double power) {  // Method: camelCase, verb
        targetPower = power;
        updateMotors();
    }
    
    /**
     * Get current power level
     * @return Current power
     */
    public double getPower() {  // Getter: starts with "get"
        return targetPower;
    }
    
    /**
     * Check if motors are running
     * @return true if running
     */
    public boolean isRunning() {  // Boolean getter: starts with "is"
        return isRunning;
    }
    
    /**
     * Update motor powers
     */
    private void updateMotors() {  // Private helper: camelCase
        leftMotor.setPower(targetPower);
        rightMotor.setPower(targetPower);
    }
}
```

---

## Common Mistakes to Avoid

| ❌ Wrong | ✅ Correct | Rule |
|---------|----------|------|
| `PracticeBot/` | `practicebot/` | Packages are lowercase |
| `drive_subsystem.java` | `DriveSubsystem.java` | Classes use PascalCase |
| `LeftMotor` | `leftMotor` | Variables use camelCase |
| `set_power()` | `setPower()` | Methods use camelCase |
| `maxPower` | `MAX_POWER` | Constants use ALL_CAPS |
| `"leftMotor"` | `"left_motor"` | Hardware names use snake_case |
| `Power()` | `setPower()` | Methods start with verbs |
| `lf` | `leftFront` | Use descriptive names |

---

## Why These Conventions Matter

1. **Consistency:** Everyone on the team uses the same style
2. **Readability:** Code is easier to understand
3. **Maintainability:** Easier to find and fix bugs
4. **Professionalism:** Matches industry standards
5. **Tool Support:** IDEs work better with proper naming
6. **Documentation:** Auto-generated docs are clearer

---

## Resources

- Official Java Naming Conventions: [Oracle Style Guide](https://www.oracle.com/java/technologies/javase/codeconventions-namingconventions.html)
- FTC SDK Documentation: Check samples in `FtcRobotController/external/samples`
- Google Java Style Guide: Additional best practices