# State Machines & Subsystems Guide
**FTC Programming Reference**

---

## What is a State Machine?

A state machine means your robot can only do **ONE main thing at a time**, and it always knows what to do next.

### Real-World Examples
- **Traffic Light**: RED → YELLOW → GREEN → RED (can't skip!)
- **Vending Machine**: Idle → Selecting → Payment → Dispensing → Idle
- **Your Robot**: IDLE → INTAKING → INDEXING → AIMING → SHOOTING → IDLE

**Key Idea**: The robot is always in exactly one state, and moves to the next state when certain conditions are met.

---

## Naming Conventions

### Subsystems (The Hardware Controllers)
**Pattern**: Use NOUNS - these are the *things*

- `IntakeSubsystem`
- `IndexerSubsystem`
- `ShooterSubsystem`
- `DrivetrainSubsystem`

**What they do**: Control the hardware (motors, servos, sensors)

### States (What the Robot is Doing)
**Pattern**: Use GERUNDS (-ing verbs) - these are *actions*

- `IDLE`
- `INTAKING`
- `INDEXING`
- `AIMING`
- `SHOOTING`

**Remember**: "The IndexerSubsystem is active during the INDEXING state"

---

## Method Naming in Subsystems

### Command Methods (Do Something)
**Pattern**: Action verbs

```java
intake()      // Start intaking
shoot()       // Fire the shooter
stop()        // Stop all motion
reset()       // Return to starting position
```

### Setters (Configure Hardware)
**Pattern**: `set` + what you're setting

```java
setPower(double power)
setAngle(double degrees)
setPosition(int ticks)
setSpeed(double rpm)
```

### Getters (Read Information)
**Pattern**: `get` + what you're reading, or `is`/`has` for boolean

```java
getPower()           // Returns current power
getAngle()           // Returns current angle
isReady()            // Returns true/false
hasGamePiece()       // Returns true/false
```

---

## State Machine Example

```java
switch(currentState) {
    case IDLE:
        // Robot is waiting
        if (driverPressedIntake()) {
            currentState = INTAKING;
        }
        break;
    
    case INTAKING:
        intakeSubsystem.intake();  // Run intake
        if (gamePieceDetected()) {
            currentState = INDEXING;
        }
        break;
    
    case INDEXING:
        indexerSubsystem.index();  // Move to shooting position
        if (gamePieceIndexed()) {
            currentState = AIMING;
        }
        break;
    
    case AIMING:
        shooterSubsystem.setAngle(targetAngle);
        shooterSubsystem.spinUp();
        if (shooterSubsystem.isReady()) {
            currentState = SHOOTING;
        }
        break;
    
    case SHOOTING:
        shooterSubsystem.shoot();
        if (shotComplete()) {
            currentState = IDLE;
        }
        break;
}
```

---

## How to Organize Subsystems

**Question**: When should mechanisms be in the same subsystem vs. separate?

### Guidelines:
1. **One subsystem per robot function** - intake, indexing, shooting, driving
2. **If it has distinct responsibilities** - separate subsystem
3. **If mechanisms always work together** - consider combining

### Our Robot Example:
- **IntakeSubsystem** - intake wheels + intake slides
- **IndexerSubsystem** - indexer mechanism
- **ShooterSubsystem** - shooter motor + turret + hood

**Why separate?** Each handles a different phase of scoring, making debugging easier!

---

## Debugging Tips

### "Where's the problem?"
With clear subsystems and states, you can quickly identify issues:

- Game piece won't intake? → Check `IntakeSubsystem` during `INTAKING`
- Game piece stuck? → Check `IndexerSubsystem` during `INDEXING`
- Shooter not firing? → Check `ShooterSubsystem` during `SHOOTING`

### State Machine Benefits:
✓ Robot behavior is predictable  
✓ Easy to add new features  
✓ Clear debugging path  
✓ Team members can work on different subsystems independently

---

## Quick Reference

| Concept | Pattern | Example |
|---------|---------|---------|
| Subsystem Class | Noun | `ShooterSubsystem` |
| State Name | Gerund (-ing) | `SHOOTING` |
| Command Method | Verb | `shoot()` |
| Setter Method | set + Property | `setPower()` |
| Getter Method | get/is/has + Property | `isReady()` |

---

**Remember**: Code organization makes teamwork easier. When everyone follows the same patterns, anyone can understand and fix any part of the robot!