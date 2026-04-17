package org.firstinspires.ftc.teamcode.Drive;

public class DriveConstants {
    public static double[] frontLeftFeedForward = {0, 1, 0};
    public static double[] frontRightFeedForward = {0, 1, 0};
    public static double[] rearLeftFeedForward = {0, 1, 0};
    public static double[] rearRightFeedForward = {0, 1, 0};

    public static double MAX_VEL = 50.98709507678554;
    public static double MAX_ACCEL = 35;
    public static double MAX_ANG_VEL = 2.3416948318481445;
    public static double MAX_ANG_ACCEL = Math.toRadians(279.98601117318435);

    public static double TICKS_PER_REV = 537.6;
    public static double MAX_RPM = 1150;

    public static boolean RUN_USING_ENCODER = false;

    public static double WHEEL_RADIUS = 1.8898; // in
    public static double GEAR_RATIO = 0.99639; // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 11.54; // in

    public static boolean frontLeftInverted = true, frontRightInverted = false, rearRightInverted = true, rearLeftInverted = false;

    public static double DEFAULT_SPEED_PERC = 1.0;
    public static double SLOW_SPEED_PERC = 0.7;

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        return 32767 / ticksPerSecond;
    }
}
