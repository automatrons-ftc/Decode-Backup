package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.GamepadExEx;
import org.firstinspires.ftc.teamcode.Hardware.MotorExEx;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

public class RobotMap {
    private GamepadExEx driverOp, toolOp;
    private MotorExEx frontLeft, rearLeft, frontRight, rearRight;
    private IMU imu;
    private HardwareMap hm;
    private List<LynxModule> hubs;
    private Telemetry telemetry;

    //// Mechanisms
    //Intake
    private MotorExEx intake;
    private ServoImplEx intakeArm;

    // Passthough
    private ServoImplEx passthoughServo;

    // Shooter
    MotorExEx wheel1, wheel2, turretMotor;
    ServoImplEx hoodServo;

    public RobotMap(HardwareMap hm, Telemetry telemetry) {
        this(hm, telemetry, null, null);
    }

    public RobotMap (HardwareMap hm, Telemetry telemetry, Gamepad driverOp,
                     Gamepad toolOp) {
        this.telemetry = telemetry;
        this.hm = hm;

        if(driverOp != null) this.driverOp = new GamepadExEx(driverOp);
        if(toolOp != null) this.toolOp = new GamepadExEx(toolOp);

        hubs = hm.getAll(LynxModule.class);

        /*--Motors--*/
        frontLeft = new MotorExEx(hm, "motorFL", Motor.GoBILDA.RPM_312);
        rearLeft = new MotorExEx(hm, "motorBL", Motor.GoBILDA.RPM_312);
        frontRight = new MotorExEx(hm, "motorFR", Motor.GoBILDA.RPM_312);
        rearRight = new MotorExEx(hm, "motorBR", Motor.GoBILDA.RPM_312);

        frontLeft.setRunMode(Motor.RunMode.RawPower);
        rearLeft.setRunMode(Motor.RunMode.RawPower);
        rearRight.setRunMode(Motor.RunMode.RawPower);
        frontRight.setRunMode(Motor.RunMode.RawPower);

        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        /*--Util--*/
        for (LynxModule module : hubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        //// ----------------------------------- Mechanisms ----------------------------------- ////
        // Intake
        intake = new MotorExEx(hm, "intake", Motor.GoBILDA.RPM_1150);

        // Passthough
        passthoughServo = hm.get(ServoImplEx.class, "intake_pivot");

        // Shooter
        wheel1 = new MotorExEx(hm, "shooter_1", Motor.GoBILDA.BARE);
        wheel2 = new MotorExEx(hm, "shooter_2", Motor.GoBILDA.BARE);
        turretMotor = new MotorExEx(hm, "rotary", Motor.GoBILDA.RPM_312);
        hoodServo = hm.get(ServoImplEx.class, "adjust_servo");
    }

    // ---------------------------------------- Gamepads ---------------------------------------- //
    public GamepadExEx getDriverOp() {
        return driverOp;
    }

    public GamepadExEx getToolOp() {
        return toolOp;
    }

    // ----------------------------------------- Motors ----------------------------------------- //
    public MotorExEx getFrontLeftMotor() {
        return frontLeft;
    }

    public MotorExEx getFrontRightMotor() {
        return frontRight;
    }

    public MotorExEx getRearLeftMotor() {
        return rearLeft;
    }

    public MotorExEx getRearRightMotor() {
        return rearRight;
    }

    // ---------------------------------------- Encoders ---------------------------------------- //


    // ------------------------------------------ Util ------------------------------------------ //
    public Telemetry getTelemetry() {
        return telemetry;
    }
    public List<LynxModule> getHubs() {
        return hubs;
    }

    // ------------------------------------------ IMU ------------------------------------------- //
    public IMU getIMU() {
        return imu;
    }

    //// ------------------------------------- Mechanisms ------------------------------------- ////
    // Intake
    public MotorExEx getIntakeMotor() {
        return intake;
    }


    public ServoImplEx getIntakeArm() {
        return intakeArm;
    }

    public ServoImplEx getPassthroughServo() {
        return passthoughServo;
    }

    // Shooter
    public MotorExEx getShooterWheel1Motor() {
        return wheel1;
    }

    public MotorExEx getShooterWheel2Motor() {
        return wheel2;
    }

    public MotorExEx getTurretMotor() {
        return turretMotor;
    }

    public ServoImplEx getHoodServo() {
        return hoodServo;
    }

    public HardwareMap getHm() {
        return hm;
    }
}
