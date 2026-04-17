package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.util.RobotLog;

public class MathFunction {

    public static double angleWrap(double angle) {
        double analogAngle = (angle / 360) % 1;

        if (analogAngle < 0) {
            return (analogAngle * 360) + 360;
        }

        return analogAngle * 360;
    }

    public static double wrapDegrees(double angleDeg) { // angle -> (-180, 180)
        angleDeg = angleDeg % 360.0;
        if (angleDeg > 180.0) {
            angleDeg -= 360.0;
        } else if (angleDeg <= -180.0) {
            angleDeg += 360.0;
        }
        return angleDeg;
    }

    public static double angleErrorWrap(double angle, double gyroValueDouble) {
        int minDistIdx, maxIdx;

        maxIdx = (int) Math.ceil(gyroValueDouble / 360);
        if (Math.abs((maxIdx - 1) * 360 + angle - gyroValueDouble) > Math.abs((maxIdx) * 360 + angle - gyroValueDouble))
            minDistIdx = maxIdx;
        else
            minDistIdx = maxIdx - 1;

        return minDistIdx * 360 + angle;
    }

    public static double dot(double[] a, double[] b) {
        if (a.length != b.length) {
            RobotLog.e("Vectors must be the same length");
        }
        double sum = 0;
        for (int i = 0; i < a.length; i++) {
            sum += a[i] * b[i];
        }
        return sum;
    }

    public static double inToMM(double in) {
        return in * 25.4;
    }

    public static double mmToIn(double mm) {
        return mm / 25.4;
    }

    public static double normalizeAngle(double angleDegrees) {
        double angle = angleDegrees % 360;
        if (angle < 0) {
            return angle + 360;
        }
        return angle;
    }

    public static double getSmallestAngleDifference(double one, double two) {
        return Math.min(normalizeAngle(one - two), normalizeAngle(two - one));
    }
}