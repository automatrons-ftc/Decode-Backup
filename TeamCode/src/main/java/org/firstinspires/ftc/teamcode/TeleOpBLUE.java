package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "BLUE TeleOP", group = "TeleOPs")
public class TeleOpBLUE extends TeleOpBase {

    @Override
    public void initialize() {
        super.initialize();
        initAllianceRelated(DecodeRobot.Alliance.BLUE, new Pose(2*24-8.25, 8.65, Math.toRadians(90)));
    }
}