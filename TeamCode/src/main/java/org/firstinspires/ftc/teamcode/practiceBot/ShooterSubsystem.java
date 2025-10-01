package org.firstinspires.ftc.teamcode.practiceBot;

//
// This subsystem will launch the balls (artifacts) from the robot
//

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterSubsystem {

    // Hardware
    public final DcMotor shooterMotor1;
    public final DcMotor shooterMotor2;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        shooterMotor1 = hardwareMap.get(DcMotor.class, "shooterMotor1");
        shooterMotor2 = hardwareMap.get(DcMotor.class, "shooterMotor2");
    }

    public void launchArtifact(){
        shooterMotor1.setPower(1.0);
        shooterMotor2.setPower(1.0);
    }

    public void launchArtifact50() {
        shooterMotor1.setPower(0.50);
        shooterMotor2.setPower(0.50);
    }
}
