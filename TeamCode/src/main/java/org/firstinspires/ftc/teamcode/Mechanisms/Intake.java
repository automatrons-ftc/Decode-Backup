package org.firstinspires.ftc.teamcode.Mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.MotorExEx;
import org.firstinspires.ftc.teamcode.RobotMap;

@Config
public class Intake extends SubsystemBase {
    private final MotorExEx intakeMotor;
    private final ServoImplEx passthough;

    private final double INTAKE_POWER = 1.0;

    public enum IntakeState {
        INTAKE,
        REVERSE,
        STOPPED
    }

    private IntakeState state = IntakeState.STOPPED;
    private Telemetry telemetry;

    private boolean passthoughEngaged = false;

    public Intake(RobotMap robotMap) {
        this.intakeMotor = robotMap.getIntakeMotor();
        intakeMotor.setZeroPowerBehavior(MotorExEx.ZeroPowerBehavior.FLOAT);
        this.passthough = robotMap.getPassthroughServo();

        this.telemetry = robotMap.getTelemetry();
    }

    @Override
    public void periodic() {
        telemetry.addData("[Intake] State ", state);
        telemetry.addData("[Passthough] Engaged ", passthoughEngaged);

        passthough.setPosition(passthoughEngaged ? 0.12 : 0.27);
    }

    public void intake() {
        intakeMotor.set(INTAKE_POWER);
        state = IntakeState.INTAKE;
    }

    public void reverse() {
        intakeMotor.set(-INTAKE_POWER);
        state = IntakeState.REVERSE;
    }

    public void stop() {
        intakeMotor.set(0);
        state = IntakeState.STOPPED;
    }

    public void engagePassthough() {
        passthoughEngaged = true;
    }

    public void disengagePassthough() {
        passthoughEngaged = false;
    }

    public boolean isPassthoughEngaged() {
        return passthoughEngaged;
    }

    public IntakeState getState() {
        return state;
    }
}
