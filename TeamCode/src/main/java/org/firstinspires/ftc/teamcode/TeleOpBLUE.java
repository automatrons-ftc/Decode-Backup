package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "BLUE TeleOP", group = "TeleOPs")
public class TeleOpBLUE extends TeleOpBase {

    @Override
    public void initialize() {
        super.initialize();
        initAllianceRelated(DecodeRobot.Alliance.BLUE);
    }
}