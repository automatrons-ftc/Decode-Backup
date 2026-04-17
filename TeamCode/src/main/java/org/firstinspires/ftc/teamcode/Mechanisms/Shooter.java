package org.firstinspires.ftc.teamcode.Mechanisms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Controllers.PIDFEx;
import org.firstinspires.ftc.teamcode.Controllers.PIDFExCon;
import org.firstinspires.ftc.teamcode.DecodeRobot;
import org.firstinspires.ftc.teamcode.Hardware.MotorExEx;
import org.firstinspires.ftc.teamcode.RobotMap;

@Config
public class Shooter extends SubsystemBase {
    private double SHOOTER_MAX_POWER = 1.0;
    private final MotorExEx wheel1, wheel2, turretMotor;
    private ServoImplEx hoodServo;

    private boolean wheelsEnabled = false;
    private boolean turretLockEnabled = false;
    private boolean hoodLockEnabled = false;

    private Supplier<Pose> curPose;
    private final Pose REDGoalPose = new Pose(141.0, 141.0, 0);
    private final Pose BLUEGoalPose = new Pose(3.0, 141.0, 0);
    private final Pose goalPose;

    private Telemetry telemetry;

    // Safety shooter positions ++ InterpolatedLUTs ++ Turret PID ++ Wheel Velo PID

    private InterpLUT wheelSpeed, hoodAngle;

    private final int MOTOR_TICKS_PER_REV = 538;
    private final double GEAR_RATIO = 130.0/28.0;
    private final int TICKS_PER_FULL_ROTATION = (int)(MOTOR_TICKS_PER_REV * GEAR_RATIO);

    private PIDFEx turretController, veloController; // TODO: Motion Profiling!!! Minimize
    private PIDFExCon coeffsTurret, coeffsVelo;
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 1, 0);

    private static final int WHEEL_TICKS_PER_REV = 28, WHEEL_MAX_RPM = 5800;
    private static final double MAX_TICKS_PER_S = 2300; // WHEEL_MAX_RPM/60.0 * 28

    private static final double MIN_HOOD = 0.9, MAX_HOOD = 0.45; // 0.45, 0.9

    public static double customTurretTarget = 0, customShooterSpeed = 0, customHoodPosition = 0.5;


    public Shooter(RobotMap robotMap, Supplier<Pose> curPose, DecodeRobot.Alliance alliance, Telemetry telemetry) {
        this.wheel1 = robotMap.getShooterWheel1Motor();
        this.wheel2 = robotMap.getShooterWheel2Motor();

        wheel1.setZeroPowerBehavior(MotorExEx.ZeroPowerBehavior.FLOAT);
        wheel1.setInverted(true);
        wheel2.setZeroPowerBehavior(MotorExEx.ZeroPowerBehavior.FLOAT);

        this.turretMotor = robotMap.getTurretMotor();
        turretMotor.resetEncoder();
        turretMotor.setZeroPowerBehavior(MotorExEx.ZeroPowerBehavior.BRAKE);
        coeffsTurret = new PIDFExCon(
                0.15,
                0.0,
                0.002,
                0.0,
                0.2,
                0.5,
                0,
                0
        );
        turretController = new PIDFEx(coeffsTurret);

        coeffsVelo = new PIDFExCon(
                14,
                0.0,
                0.005,
                0.0,
                0.0,
                10,
                600,
                0.8
        );
        veloController = new PIDFEx(coeffsVelo);

        this.hoodServo = robotMap.getHoodServo();
        hoodServo.setPosition(Range.scale(1, 0, 1, MIN_HOOD, MAX_HOOD));

//        this.telemetry = robotMap.getTelemetry();
        this.telemetry = telemetry;
        this.curPose = curPose;

        // Select Correct Goal Based On Alliance
        goalPose = (alliance == DecodeRobot.Alliance.RED) ? REDGoalPose : BLUEGoalPose;

        // Initialize LUTs here
        wheelSpeed = new InterpLUT();
        hoodAngle = new InterpLUT();

        wheelSpeed.add(40, 0.64);
        wheelSpeed.add(70, 0.75);
        wheelSpeed.add(92, 0.82);
        wheelSpeed.add(114, 0.88);
        wheelSpeed.add(135, 0.99);
        wheelSpeed.add(137, 0.99);
        wheelSpeed.add(139, 1);

        hoodAngle.add(40, 0.0);
        hoodAngle.add(70, 0.5);
        hoodAngle.add(92, 0.7);
        hoodAngle.add(114, 0.9);
        hoodAngle.add(135, 0.95);
        hoodAngle.add(137, 0.95);

        wheelSpeed.createLUT();
        hoodAngle.createLUT();
    }

    @Override
    public void periodic() {
        turretController.setSetPoint(Range.clip(wheelsEnabled ? getAngleToGoal() : 0, -120, 60));
        turretMotor.set(turretController.calculate(getTurretAngle()));

        telemetry.addData("[Shooter] Wheel State ", wheelsEnabled);
        telemetry.addData("[Shooter] Turret Lock ", turretLockEnabled);
        telemetry.addData("[Shooter] Hood Lock ", hoodLockEnabled);
        telemetry.addData("[Shooter] Turret Pos: ", getTurretAngle());
        telemetry.addData("[Shooter] Goal Dist: ", getDistanceToGoal());

        FtcDashboard.getInstance().getTelemetry().addData("Actual Velo : ", wheel1.getVelocity());
        FtcDashboard.getInstance().getTelemetry().addData("Target Velo: ", customShooterSpeed * 0.9 * MAX_TICKS_PER_S);
        FtcDashboard.getInstance().getTelemetry().addData("Turret Target: ", customTurretTarget);
        FtcDashboard.getInstance().getTelemetry().addData("Turret Angle: ", getTurretAngle());

        if(!inLUTRange()) return;

        if(wheelsEnabled) {
            wheel1.set(getControlledWheelPower(wheelSpeed.get(getDistanceToGoal())));
            wheel2.set(getControlledWheelPower(wheelSpeed.get(getDistanceToGoal())));
        }

        hoodServo.setPosition(Range.scale(wheelSpeed.get(getDistanceToGoal()), 0, 1, MIN_HOOD, MAX_HOOD));
    }

    public double getControlledWheelPower(double power) {
        double speed = 0.9 * power * MAX_TICKS_PER_S;
        veloController.setSetPoint(speed);
        double velocity = veloController.calculate(wheel1.getCorrectedVelocity()) + feedforward.calculate(speed, wheel1.getAcceleration());
        return velocity / MAX_TICKS_PER_S;
    }

    public boolean wheelsAtSpeed() {
        return Math.abs(veloController.getPositionError()) < 100;
    }

    public boolean turretAtTarget() {
        return Math.abs(turretController.getPositionError()) < 2;
    }

    public boolean inLUTRange() {
        return getDistanceToGoal() > 43 && getDistanceToGoal() < 138;
    }

    public void enableWheels() {
        wheelsEnabled = true;
    }

    public void disableWheels() {
        wheel1.set(0);
        wheel2.set(0);
        wheelsEnabled = false;
    }

    public boolean areWheelsEnabled() {
        return wheelsEnabled;
    }

    public double getTurretAngle() {
        return ((turretMotor.getCurrentPosition())%TICKS_PER_FULL_ROTATION)*360.0/TICKS_PER_FULL_ROTATION;
    }

    public boolean turretInRange() {
        return getTurretAngle() > -120 && getTurretAngle() < 72;
    }


    // IK Stuff
    private double getDistanceToGoal() {
        Pose pose = curPose.get();
        double dx = goalPose.getX() - pose.getX();
        double dy = goalPose.getY() - pose.getY();
        return Math.hypot(dx, dy);
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

    public double getAngleToGoal() {
        Pose pose = curPose.get();
        double dx = goalPose.getX() - pose.getX();
        double dy = goalPose.getY() - pose.getY();

        double robotHeading = Math.toDegrees(curPose.get().getHeading()) % 360;
        if (robotHeading >= 180) robotHeading -= 360;
        if (robotHeading < -180) robotHeading += 360;

        double relativeAngle = Math.toDegrees(Math.atan2(dy, dx)) - robotHeading;
        relativeAngle %= 360;
        if (relativeAngle >= 180) relativeAngle -= 360;
        if (relativeAngle < -180) relativeAngle += 360;

        return relativeAngle;
    }
}
