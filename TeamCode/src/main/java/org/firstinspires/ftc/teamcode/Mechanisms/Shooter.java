package org.firstinspires.ftc.teamcode.Mechanisms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.CodeParameters;
import org.firstinspires.ftc.teamcode.Controllers.MotorFF;
import org.firstinspires.ftc.teamcode.Controllers.PIDFEx;
import org.firstinspires.ftc.teamcode.Controllers.PIDFExCoeffs;
import org.firstinspires.ftc.teamcode.Controllers.StateMachine;
import org.firstinspires.ftc.teamcode.DecodeRobot;
import org.firstinspires.ftc.teamcode.Hardware.MotorExEx;
import org.firstinspires.ftc.teamcode.RobotMap;

import java.util.function.DoubleSupplier;

@Config
public class Shooter extends SubsystemBase {
    // ---------------------------------------- Hardware ---------------------------------------- //
    private MotorExEx wheel1, wheel2;
    private ServoImplEx hoodServo, shooterFinger;
    private MotorExEx turretMotor;

    // ---------------------------------------- Constants --------------------------------------- //
    // Wheel
    private static final int WHEEL_TICKS_PER_REV = 28, WHEEL_MAX_RPM = 5800;
    private static final double MAX_TICKS_PER_S = 2700;

    // Turret
    private final int MOTOR_TICKS_PER_REV = 538;
    private final double GEAR_RATIO = 130.0/28.0;
    private final int TICKS_PER_FULL_ROTATION = (int)(MOTOR_TICKS_PER_REV * GEAR_RATIO);

    // IK (Vectoring)
    private static final double
        stationaryScale = 1.0,
        robotVelocityScale = 0.00492,
        robotVelocityScaleAuto = 0.003,
        wheelSpeedFactor = 1.0;

    private double poseEstimation_dt = 0.21;

    // ----------------------------------------- States ----------------------------------------- //
    private boolean wheelsEnabled = false;
    private boolean hoodLockEnabled = true;
    private boolean small_triangle_accel = false;
    public static double accel_value = 0.95;

    public enum ShooterGoal {
        ALLIANCE_GOAL,
        OBELISK,
        AUTO_CUSTOM,
        DISABLED
    }

    private ShooterGoal shooterLock;

    // ---------------------------------------- Poses ------------------------------------------- //
    private Supplier<Pose> curPose, curPoseVel, futurePose, curPoseAccel;
    private final Pose goalPose;

    // ---------------------------------- Controllers and LUTs ---------------------------------- //
    private LookUpValues lu_values;
    private PIDFEx turretController, veloController;
    private PIDFExCoeffs coeffsTurret, coeffsVelo;
    private MotorFF feedforward = new MotorFF(0.02, 1.02, 0.12);
    private boolean inAuto = false;

    // ------------------------------------ Turret Zeroing -------------------------------------- //
    private boolean turretZeroed = false;
    private double turretZeroPower = -0.25;
    private boolean startReversed = false;
    private StateMachine hasStalled;

    private double auto_custom_angle = 0.0;

    private boolean accelWheel = false;

    // ------------------------------------------ Util ------------------------------------------ //
    private Telemetry telemetry;
    private DoubleSupplier voltage;

    private boolean fingerOpen = false;

    public Shooter(RobotMap robotMap, Supplier<Pose> curPose, Supplier<Pose> curPoseVel,
                   DecodeRobot.Alliance alliance, boolean doZero) {

        this(robotMap, curPose, curPoseVel, () -> new Pose(0,0,0), alliance, doZero, false, false);
    }

    public Shooter(RobotMap robotMap, Supplier<Pose> curPose, Supplier<Pose> curPoseVel, Supplier<Pose> curPoseAccel,
                   DecodeRobot.Alliance alliance, boolean doZero, boolean startReversed,
                   boolean inAuto
    ) {
        this(robotMap, curPose, curPoseVel, curPoseAccel, alliance, doZero, startReversed, inAuto, false);
    }

    public Shooter(RobotMap robotMap, Supplier<Pose> curPose, Supplier<Pose> curPoseVel, Supplier<Pose> curPoseAccel,
                   DecodeRobot.Alliance alliance, boolean doZero, boolean startReversed,
                   boolean inAuto, boolean accelWheel
    ) {
        this.wheel1 = robotMap.getShooterWheel1Motor();
        this.wheel2 = robotMap.getShooterWheel2Motor();

        this.hoodServo = robotMap.getHoodServo();
        this.shooterFinger = robotMap.getShooterFinger();
        closeFinger();

        this.turretMotor = robotMap.getTurretMotor();
        turretMotor.setInverted(false);
        turretMotor.resetEncoder();
        turretZeroed = !doZero;
        turretZeroed = true;
        this.startReversed = !doZero && startReversed;
        this.inAuto = inAuto;
        this.accelWheel = accelWheel;

        this.telemetry = robotMap.getTelemetry();

        this.curPose = curPose;
        this.futurePose = () -> estimateFuturePose(poseEstimation_dt);
        this.curPoseVel = curPoseVel;
        this.curPoseAccel = curPoseAccel;

        shooterLock = ShooterGoal.ALLIANCE_GOAL;

        hasStalled = new StateMachine(() -> ((DcMotorEx)turretMotor.getRawMotor()).getCurrent(CurrentUnit.AMPS) > CodeParameters.turretZeroCurrentThreshold, 300);

        wheel1.setZeroPowerBehavior(MotorExEx.ZeroPowerBehavior.FLOAT);
        wheel2.setZeroPowerBehavior(MotorExEx.ZeroPowerBehavior.FLOAT);
        wheel2.setInverted(true);

        goalPose = (alliance == DecodeRobot.Alliance.RED) ? CodeParameters.REDGoalPose : CodeParameters.BLUEGoalPose;

        coeffsTurret = new PIDFExCoeffs(
                0.055,
                0.13,
                0.00225,
                0.0,
                0.2,
                0.0,
                20,
                0.6
        );

        turretController = new PIDFEx(coeffsTurret);

        coeffsVelo = new PIDFExCoeffs(
                23,
                0.0,
                0.05,
                0.0,
                0.0,
                3,
                600,
                0.8
        );
        veloController = new PIDFEx(coeffsVelo);

        lu_values = new LookUpValues(LookUpValues.CurrentWheel.BRONZE_HEAVY);
        lu_values.fiilWithValues();

        poseEstimation_dt = inAuto ? 0.34 : 0.21;

        enableWheels();
    }

    @Override
    public void periodic() {
//        hoodServo.setPosition(Range.scale(
//                0,
//                0,
//                1,
//                CodeParameters.MIN_HOOD_POS,
//                CodeParameters.MAX_HOOD_POS
//        ));

        if(!turretZeroed) {
            hasStalled.update();
            turretMotor.set(turretZeroPower);
            if(hasStalled.isJustActive()) {
                turretMotor.resetEncoder();
                turretMotor.set(0);
                turretZeroed = true;
            }
            return;
        }

        if(shooterLock == ShooterGoal.DISABLED) {
            turretController.setSetPoint(-101);
            turretMotor.set(Range.clip(
                    turretController.calculate(getTurretAngle()),
                    -CodeParameters.MAX_TURRET_POWER,
                    CodeParameters.MAX_TURRET_POWER
            ));

            hoodServo.setPosition(CodeParameters.MIN_HOOD_POS);
            return;
        }

        // ------------------------------------- Telemetry -------------------------------------- //
        telemetry.addData("[Shooter] Wheel State ", wheelsEnabled);
        telemetry.addData("[Shooter] Hood Lock ", hoodLockEnabled);
        telemetry.addData("[Shooter] Shooter ", shooterLock);
        telemetry.addData("[Shooter] Turret Angle: ", getTurretAngle());
        telemetry.addData("[Shooter] Turret Ticks: ", turretMotor.getCurrentPosition());
        telemetry.addData("[Shooter] GOAL Dist: ", getDistanceToGoal(futurePose.get()));
        telemetry.addData("[Shooter] GOAL Angle: ", getAngleToGoal());

        // --------------------------------------- Turret --------------------------------------- //
        Vector curShootingVector = calcShootingVector();
        if(shooterLock == ShooterGoal.ALLIANCE_GOAL) {
            turretController.setSetPoint(wheelsEnabled ? getTurretTarget(curShootingVector) : 0);
        }

        if(shooterLock == ShooterGoal.AUTO_CUSTOM) {
            turretController.setSetPoint(auto_custom_angle);
        }

        turretMotor.set(Range.clip(
                turretController.calculate(getTurretAngle()),
                -CodeParameters.MAX_TURRET_POWER,
                CodeParameters.MAX_TURRET_POWER
        ));

//        if(wheelsEnabled) {
//            wheel1.set(getControlledWheelPower(0.6));
//            wheel2.set(getControlledWheelPower(0.6));
//        }

        if(!inLUTRange()) {
            if(inAuto) {
                wheel1.set(getControlledWheelPower(0.58));
                wheel2.set(getControlledWheelPower(0.58));
            }
            return;
        }

        if(!inLUTRange()) return;

        // ---------------------------------------- Hood ---------------------------------------- //
        hoodServo.setPosition(Range.scale(
                (hoodLockEnabled ? lu_values.getHood(getDistanceToGoal(futurePose.get())) : 0),
                0,
                1,
                CodeParameters.MIN_HOOD_POS,
                CodeParameters.MAX_HOOD_POS
        ));

        // --------------------------------------- Wheels --------------------------------------- //
        if(wheelsEnabled) {
            if(small_triangle_accel && atSmallTriangle()) {
                wheel1.set(getControlledWheelPower(accel_value));
                wheel2.set(getControlledWheelPower(accel_value));
                return;
            }

            double futurePoseDist = getDistanceToGoal(futurePose.get());
            wheel1.set(getControlledWheelPower(lu_values.getWheel(futurePoseDist)));
            wheel2.set(getControlledWheelPower(lu_values.getWheel(futurePoseDist)));
        }
    }

    // ----------------------------------------- Wheels ----------------------------------------- //
    public double getControlledWheelPower(double power) {
        double speed = 0.9 * power * MAX_TICKS_PER_S;
        veloController.setSetPoint(speed);
        double velocity = veloController.calculate(-wheel1.getCorrectedVelocity()) +
                feedforward.calculate(speed, -wheel1.getAcceleration());

        FtcDashboard.getInstance().getTelemetry().addData("Target VEL: ", speed);
        FtcDashboard.getInstance().getTelemetry().addData("Actual VEL: ", wheel1.getCorrectedVelocity());
        FtcDashboard.getInstance().getTelemetry().update();

        return velocity / MAX_TICKS_PER_S;
    }

    public boolean accelVelTarget() {
        return wheel1.getCorrectedVelocity() > 800;
    }

    public void enableWheels() {
        wheelsEnabled = true;
    }

    public void disableWheels() {
        wheelsEnabled = false;
        wheel1.set(0);
        wheel2.set(0);
    }

    public boolean areWheelsEnabled() {
        return wheelsEnabled;
    }

    public boolean wheelsAtSpeed() {
        return Math.abs(veloController.getPositionError()) < 90;
    }

    // ----------------------------------------- Turret ----------------------------------------- //
    public double getTurretAngle() {
//        if (startReversed) {
//            return (turretMotor.getCurrentPosition()*(360.0/TICKS_PER_FULL_ROTATION))*CodeParameters.TURRET_ANGLE_MULTIPLIER - CodeParameters.turretZeroOffsetReversed;
//        }
//
//        return (turretMotor.getCurrentPosition()*(360.0/TICKS_PER_FULL_ROTATION))*CodeParameters.TURRET_ANGLE_MULTIPLIER - CodeParameters.turretZeroOffset;

        // simpler
        return (turretMotor.getCurrentPosition()*(360.0/TICKS_PER_FULL_ROTATION))*CodeParameters.TURRET_ANGLE_MULTIPLIER -
                (startReversed ? CodeParameters.turretZeroOffsetReversed : CodeParameters.turretZeroOffset);
    }

    public void resetOffset() {
        CodeParameters.turretZeroOffset = startReversed ? -CodeParameters.turretZeroOffsetReversed : -CodeParameters.turretZeroOffset;
    }

    public boolean turretInRange() {
        double angleToGoal = getAngleToGoal();
        return angleToGoal > CodeParameters.MIN_TURRET_ANGLE && angleToGoal < CodeParameters.MAX_TURRET_ANGLE;
    }

    public boolean turretAtGoal() {
        return Math.abs(turretController.getPositionError()) < (atSmallTriangle() ? 1.2 : 0.7);
    }

    public boolean turretAtGoal(double threshold) {
        return Math.abs(turretController.getPositionError()) < threshold;
    }

    // ---------------------------------------- IK Stuff ---------------------------------------- //
    public double getDistanceToGoal(Pose posaki) {
        double dx = goalPose.getX() - posaki.getX();
        double dy = goalPose.getY() - posaki.getY();
        return Math.hypot(dx, dy);
    }

    public double getDistanceToGoal() {
        double dx = goalPose.getX() - curPose.get().getX();
        double dy = goalPose.getY() - curPose.get().getY();
        return Math.hypot(dx, dy);
    }

    public double getAngleToGoal() { // true -> goal, false -> obelisk
        double dx = goalPose.getX() - curPose.get().getX();
        double dy = goalPose.getY() - curPose.get().getY();

//        double targetAngle_ref = Math.toDegrees(Math.atan2(dy_ref, dx_ref));
//        double dx = dx_ref;
//        double dy = dy_ref;

//        if(Math.abs(targetAngle_ref) > 70.0) dx += Range.scale(targetAngle_ref, -70.0, -90.0, -1.5, -0.8);
//        if(atSmallTriangle()) dy += 0;

        return -Math.toDegrees(Math.atan2(dx, dy));
    }

    public double getTurretTarget(Vector shooting) {
        double robotHeading = (Math.toDegrees(curPose.get().getHeading() - 90) % 360);
        if (robotHeading >= 180) robotHeading -= 360;
        if (robotHeading < -180) robotHeading += 360;

        double relativeAngle = Math.toDegrees(shooting.getAngle()) - robotHeading;
        relativeAngle %= 360;
        if (relativeAngle >= 180) relativeAngle -= 360;
        if (relativeAngle < -180) relativeAngle += 360;

        relativeAngle = Range.clip(relativeAngle, CodeParameters.MIN_TURRET_ANGLE, CodeParameters.MAX_TURRET_ANGLE);

//        return relativeAngle;
        return 0;
    }

    public Pose estimateFuturePose(double dt) {
        Vector robotVelocityVec = new Vector(curPoseVel.get().getX(), curPoseVel.get().getY(), false);

        if(robotVelocityVec.getMagnitude() < 0.5) return curPose.get();

        Vector robotAccelVec = new Vector(curPoseAccel.get().getX(), curPoseAccel.get().getY(), false);

        Vector Ut = VectorMath.scale_vector(robotVelocityVec, dt);

        Vector At = VectorMath.scale_vector(robotAccelVec, 0.5 * (dt * dt));

        Vector futureVector = VectorMath.add_vectors(
                new Vector(
                    curPose.get().getX(),
                    curPose.get().getY(), false
                ),
                Ut,
                At
        );

        return new Pose(futureVector.getVx(), futureVector.getVy());
    }

    public Vector calcShootingVector() {
        if(!inLUTRange()) {
            return new Vector(1, Math.toRadians(getAngleToGoal()), true);
        }

        double stationary_angle = Math.toRadians(getAngleToGoal());
        double artifact_velocity = lu_values.getWheel(getDistanceToGoal(futurePose.get()));

        Vector stationaryVec = new Vector(artifact_velocity, stationary_angle, true);
        Vector robotVelocityVec = new Vector(curPoseVel.get().getX(), curPoseVel.get().getY(), false);

        Vector scaledStationaryVec = VectorMath.scale_vector(stationaryVec, stationaryScale);

        Vector scaledRobotVelocityVec;

        if (!inAuto) {
            scaledRobotVelocityVec = VectorMath.scale_vector(robotVelocityVec, robotVelocityScale);
        } else {
            scaledRobotVelocityVec = VectorMath.scale_vector(robotVelocityVec, robotVelocityScaleAuto);
        }

        telemetry.addData("[Shooter] Stationary Vector (r, θ): ", "(%.2f, %.2f)", stationaryVec.getMagnitude(), Math.toDegrees(stationaryVec.getAngle()));
        telemetry.addData("[Shooter] Velocity Vector (x, y): ", "(%.2f, %.2f)", robotVelocityVec.getVx(), robotVelocityVec.getVy());
        telemetry.addData("[Shooter] Velocity Vector (r, θ): ", "(%.2f, %.2f)", robotVelocityVec.getMagnitude(), Math.toDegrees(robotVelocityVec.getAngle()));

        if(robotVelocityVec.getMagnitude() < 2.0) return stationaryVec;

        return VectorMath.scale_vector(
                VectorMath.subtract_vectors(scaledStationaryVec, scaledRobotVelocityVec),
                wheelSpeedFactor
        );
    }

    public boolean inLUTRange() {
        if (inAuto) {
            return getDistanceToGoal(futurePose.get()) > 24 && getDistanceToGoal(futurePose.get()) < 164.5;
        }

        return lu_values.inRange(getDistanceToGoal(futurePose.get()));
    }

    public boolean atSmallTriangle() {
        return getDistanceToGoal(futurePose.get()) > (325.0/2.54);
    }

    public void zeroTurret() {
        turretZeroed = false;
    }

    public void enableParkingState() {
        shooterLock = ShooterGoal.DISABLED;
    }

    public void disableParkingState() {
        shooterLock = ShooterGoal.ALLIANCE_GOAL;
    }

    public boolean isParked(){
        return shooterLock == ShooterGoal.DISABLED;
    }

    public void increase_turret_offset() { CodeParameters.turretZeroOffset += 1.0; }
    public void decrease_turret_offset() { CodeParameters.turretZeroOffset -= 1.0; }

    public void enableObelisk() {
        //pare mou mia pipa
    }

    public void disableObelisk() {
        //pare mou mia pipa
    }

    public void enableAutoCustom(double customAngle) {
        shooterLock = ShooterGoal.AUTO_CUSTOM;
        auto_custom_angle = customAngle;
    }

    public void enableSmallTriangleAccel() {
        small_triangle_accel = true;
    }

    public void disableSmallTriangleAccel() {
        small_triangle_accel = false;
    }

    public void closeFinger() {
        fingerOpen = false;
        shooterFinger.setPosition(CodeParameters.FINGER_CLOSED_POS);
    }

    public void openFinger() {
        fingerOpen = true;
        shooterFinger.setPosition(CodeParameters.FINGER_OPENED_POS);
    }

    public boolean isFingerOpen() {
        return fingerOpen;
    }
}
