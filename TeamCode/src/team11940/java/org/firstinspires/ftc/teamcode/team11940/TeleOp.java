package org.firstinspires.ftc.teamcode.team11940;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.common.config.RobotConfig;
import org.firstinspires.ftc.teamcode.common.teleop.PrimeTeleOpBase;

@TeleOp(name = "Team 11940 TeleOp", group = "TeleOp")
public class PrimeTeleOp extends PrimeTeleOpBase {
    @Override
    protected RobotConfig getRobotConfig() { return new Team11940Config(); }
}
