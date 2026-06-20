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
    public static double MIN_HOOD_CORRECTION = 0.0, MAX_HOOD_CORRECTION = 0.125;

    // ------------------------------------ Turret Constants ------------------------------------ //
    public static double MAX_TURRET_POWER = 1.0;
    public static double MIN_TURRET_ANGLE = -143.0, MAX_TURRET_ANGLE = 143.0;
    public static double TURRET_ANGLE_MULTIPLIER = 1.0;
    public static double turretZeroCurrentThreshold = 1.6;
    public static double turretZeroOffset = -146.0;
    public static double turretZeroOffsetReversed = -195.5346;

    // ----------------------------- Goal Poses for Auto Targeting ------------------------------ //
    public static Pose REDGoalPose = new Pose(139, 141.0, 0);
    public static Pose BLUEGoalPose = new Pose(5.0, 141.0, 0);

    // ----------------------------------------- Finger ----------------------------------------- //
    public static double FINGER_OPENED_POS = 0.9, FINGER_CLOSED_POS = 0.72;

    // --------------------------------------- Indicator ---------------------------------------- //
    public static double BLACK_COLOR = 0.0, RED_COLOR = 0.3, GREEN_COLOR = 0.47;

    ////// ----------------------------------- Autonomous ----------------------------------- //////
    public static long GATE_WAIT_TIME = 2200;
    public static double GATE_SHOOTING_TOLERANCE = 7.5;
    // ---------------------------------------- RED GATE ---------------------------------------- //
    public static Pose RED_START_POSE = new Pose(118, 126, Math.toRadians(46));
    public static Pose RED_SHOOT_PRELOADS_POSE = new Pose(91.4, 92.3, Math.toRadians(46));
    public static Pose RED_STACK2_POSE = new Pose(132.4, 59, 0);
    public static Pose RED_OPEN_ALLY = new Pose(124.8, 63.2, 0);
    public static Pose RED_GATE_POSE = new Pose(131.2, 59.6, Math.toRadians(35));
    public static Pose RED_SHOOT_POSE = new Pose(87, 80, Math.toRadians(322));
    public static Pose RED_STACK1_POSE = new Pose(125, 83.4, 0);
    public static Pose RED_PARK_POSE = new Pose(89, 108, Math.toRadians(305));
    public static double[] RED_GATE_TURRET_ANGLES = {1, 42, 83, 83, 83, 83}; // PRELOADS, STACK2, GATE, GATE, GATE, PARK

    // ---------------------------------------- BLUE GATE --------------------------------------- //
    public static Pose BLUE_START_POSE = new Pose(144-118, 126, Math.toRadians(134));
    public static Pose BLUE_SHOOT_PRELOADS_POSE = new Pose(144-91.4, 92.3, Math.toRadians(134));
    public static Pose BLUE_STACK2_POSE = new Pose(144-132.4, 59,  Math.toRadians(180));
    public static Pose BLUE_OPEN_ALLY = new Pose(144-125, 65,  Math.toRadians(180));
    public static Pose BLUE_GATE_POSE = new Pose(144-132.3, 61.3, Math.toRadians(142));
    public static Pose BLUE_SHOOT_POSE = new Pose(144-87, 80, Math.toRadians(218));
    public static Pose BLUE_STACK1_POSE = new Pose(144-125, 86,  Math.toRadians(180));
    public static Pose BLUE_PARK_POSE = new Pose(144-89, 108, Math.toRadians(235));
    public static double[] BLUE_GATE_TURRET_ANGLES = {-1, -45, -85, -85, -85, -85}; // PRELOADS, STACK2, GATE, GATE, GATE, PARK

    // ---------------------------------------- RED FAR ----------------------------------------- //
    public static Pose RED_START_POSE_FAR = new Pose(90, 8.25, Math.toRadians(90));
    public static Pose RED_SHOOT_PRELOADS_POSE_FAR = new Pose(96-10.25, 22, Math.toRadians(90));
    public static Pose RED_STACK3_POSE = new Pose(132.2, 36, 0);
    public static Pose RED_SHOOT_POSE_FAR = new Pose(86, 21, 0);
    public static Pose RED_HP_POSE = new Pose(132.2, 9.8, 0);
    public static Pose RED_PARK_POSE_FAR = new Pose(92, 26, 0);
    public static double[] RED_FAR_TURRET_ANGLES = {-23, 67, 67, 67, 67, 67.5}; // PRELOADS, STACK3, HP, HP, HP, HP

    // ---------------------------------------- BLUE FAR ----------------------------------------- //
    public static Pose BLUE_START_POSE_FAR = new Pose(144-90, 8.25, Math.toRadians(90));
    public static Pose BLUE_SHOOT_PRELOADS_POSE_FAR = new Pose(144-85.75, 22, Math.toRadians(90));
    public static Pose BLUE_STACK3_POSE = new Pose(144-132.2, 36, Math.toRadians(180));
    public static Pose BLUE_SHOOT_POSE_FAR = new Pose(144-86, 21, Math.toRadians(180));
    public static Pose BLUE_HP_POSE = new Pose(144-132.2, 9.4, Math.toRadians(180));
    public static Pose BLUE_PARK_POSE_FAR = new Pose(144-92, 26, Math.toRadians(180));
    public static double[] BLUE_FAR_TURRET_ANGLES = {25.2, -65, -65, -65, -65, -65}; // PRELOADS, STACK3, HP, HP, HP, HP
}
