package org.firstinspires.ftc.teamcode.team11940;

import org.firstinspires.ftc.teamcode.common.config.RobotConfig;
import org.firstinspires.ftc.teamcode.common.teleop.TeleOpBase;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Team 11940 TeleOp", group = "TeleOp")
public class TeleOp extends TeleOpBase {
    @Override
    protected RobotConfig getRobotConfig() { return new Team11940Config(); }
}
