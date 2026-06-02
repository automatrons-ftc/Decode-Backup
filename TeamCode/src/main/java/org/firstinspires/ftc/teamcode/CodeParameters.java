package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

@Config
public class CodeParameters {
    ////// ----------------------------------- Mechanisms ----------------------------------- //////
    //// ---------------------------------- Intake Constants ---------------------------------- ////
    public static double PASSTHROUGH_ENGAGED_POS = 0.75, PASSTHROUGH_DISENGAGED_POS = 0.6;
    public static double INTAKE_POWER = 1.0;

    //// --------------------------------- Shooter Constants ---------------------------------- ////
    // ------------------------------------- Hood Constants ------------------------------------- //
    public static double MIN_HOOD_POS = 0.9, MAX_HOOD_POS = 0.45;

    // ------------------------------------ Turret Constants ------------------------------------ //
    public static double MAX_TURRET_POWER = 0.7;
    public static double MIN_TURRET_ANGLE = -143.0, MAX_TURRET_ANGLE = 143.0;
    public static double TURRET_ANGLE_MULTIPLIER = 1.0;
    public static double turretZeroCurrentThreshold = 1.6;
    public static double turretZeroOffset = -146.10;
    public static double turretZeroOffsetReversed = -195.5346;

    // ----------------------------- Goal Poses for Auto Targeting ------------------------------ //
    public static Pose REDGoalPose = new Pose(141.0, 141.0, 0);
    public static Pose BLUEGoalPose = new Pose(3.0, 141.0, 0);

    // ----------------------------------------- Finger ----------------------------------------- //
    public static double FINGER_OPENED_POS = 0.918, FINGER_CLOSED_POS = 0.755;


    ////// ----------------------------------- Autonomous ----------------------------------- //////

}
