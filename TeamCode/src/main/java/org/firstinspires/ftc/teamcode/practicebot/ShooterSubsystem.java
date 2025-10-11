package org.firstinspires.ftc.teamcode.practicebot;

//
// This subsystem will launch the balls (artifacts) from the robot
//

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterSubsystem {

    // Hardware
    public final DcMotor shooterMotor1;
    public final DcMotor shooterMotor2;
    
    // Power level control (0.0 to 1.0)
    private double targetPower = 0.0;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        shooterMotor1 = hardwareMap.get(DcMotor.class, "shooterMotor1");
        shooterMotor2 = hardwareMap.get(DcMotor.class, "shooterMotor2");
        
        // Set motors to spin in opposite directions
        shooterMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void launchArtifact(){
        shooterMotor1.setPower(1.0);
        shooterMotor2.setPower(1.0);
    }

    public void launchArtifact50() {
        shooterMotor1.setPower(0.50);
        shooterMotor2.setPower(0.50);
    }
    
    // Increase power by 10%
    public void increasePower() {
        targetPower = Math.min(targetPower + 0.1, 1.0);
    }
    
    // Decrease power by 10%
    public void decreasePower() {
        targetPower = Math.max(targetPower - 0.1, 0.0);
    }
    
    // Run motors at target power
    public void runShooter() {
        shooterMotor1.setPower(targetPower);
        shooterMotor2.setPower(targetPower);
    }
    
    // Stop motors
    public void stopShooter() {
        shooterMotor1.setPower(0.0);
        shooterMotor2.setPower(0.0);
    }
    
    // Get current target power level
    public double getTargetPower() {
        return targetPower;
    }
}
