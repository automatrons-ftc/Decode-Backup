package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "RED TeleOP", group = "TeleOPs")
public class TeleOpRED extends TeleOpBase {

    @Override
    public void initialize() {
        super.initialize();
        initAllianceRelated(DecodeRobot.Alliance.RED);
    }
}