package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "RED TeleOP", group = "TeleOPs")
public class TeleOpRED extends TeleOpBase {

    @Override
    public void initialize() {
        super.initialize();
        initAllianceRelated(DecodeRobot.Alliance.RED, new Pose(8.25+4*24, 8.65, Math.toRadians(90)));
    }
}