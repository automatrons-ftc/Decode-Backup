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
    public static double MIN_HOOD_CORRECTION = 0.0, MAX_HOOD_CORRECTION = 0.115;

    // ------------------------------------ Turret Constants ------------------------------------ //
    public static double MAX_TURRET_POWER = 1.0;
    public static double MIN_TURRET_ANGLE = -143.0, MAX_TURRET_ANGLE = 143.0;
    public static double TURRET_ANGLE_MULTIPLIER = 1.0;
    public static double turretZeroCurrentThreshold = 1.6;
    public static double turretZeroOffset = -147.0;
    public static double turretZeroOffsetReversed = -195.5346;

    // ----------------------------- Goal Poses for Auto Targeting ------------------------------ //
    public static Pose REDGoalPose = new Pose(141.0, 141.0, 0);
    public static Pose BLUEGoalPose = new Pose(3.0, 141.0, 0);

    // ----------------------------------------- Finger ----------------------------------------- //
    public static double FINGER_OPENED_POS = 0.9, FINGER_CLOSED_POS = 0.72;

    // --------------------------------------- Indicator ---------------------------------------- //
    public static double BLACK_COLOR = 0.0, RED_COLOR = 0.3, GREEN_COLOR = 0.47;

    ////// ----------------------------------- Autonomous ----------------------------------- //////
    public static long GATE_WAIT_TIME = 1500;
    // ---------------------------------------- RED GATE ---------------------------------------- //
    public static Pose RED_START_POSE = new Pose(118, 126, Math.toRadians(46));
    public static Pose RED_SHOOT_PRELOADS_POSE = new Pose(91.4, 92.3, Math.toRadians(46));
    public static Pose RED_STACK2_POSE = new Pose(133.2, 59, 0);
    public static Pose RED_GATE_POSE = new Pose(131.2, 60, Math.toRadians(35));
    public static Pose RED_SHOOT_POSE = new Pose(87, 80, Math.toRadians(322));
    public static Pose RED_STACK1_POSE = new Pose(125, 83.4, 0);
    public static Pose RED_PARK_POSE = new Pose(89, 108, Math.toRadians(305));
    public static double SHOOTING_TOLERANCE = 7.5;
}
