package org.firstinspires.ftc.teamcode.Mechanisms;

import org.firstinspires.ftc.teamcode.CodeParameters;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Controllers.StateMachine;
import org.firstinspires.ftc.teamcode.Hardware.MotorExEx;
import org.firstinspires.ftc.teamcode.RobotMap;

@Config
public class Intake extends SubsystemBase {
    private final MotorExEx intakeMotor;
    private final ServoImplEx passthough;
    private final DigitalChannel beam;

    public enum IntakeState {
        INTAKE,
        REVERSE,
        STOPPED
    }

    private IntakeState state = IntakeState.STOPPED;
    private Telemetry telemetry;
    private boolean passthoughEngaged = false;

    private StateMachine intakeGotFull;

    public Intake(RobotMap robotMap) {
        this.intakeMotor = robotMap.getIntakeMotor();
        intakeMotor.setZeroPowerBehavior(MotorExEx.ZeroPowerBehavior.FLOAT);

        this.passthough = robotMap.getPassthroughServo();

        this.beam = robotMap.getIntakeBeam();
        this.intakeGotFull = new StateMachine(() -> !beam.getState(), 350);

        this.telemetry = robotMap.getTelemetry();
    }

    @Override
    public void periodic() {
        telemetry.addData("[Intake] State ", state);
        telemetry.addData("[Passthough] Engaged ", passthoughEngaged);
        telemetry.addData("[Intake] Beam Breaked? ", beam.getState());

        intakeGotFull.update();

        passthough.setPosition(passthoughEngaged ? CodeParameters.PASSTHROUGH_ENGAGED_POS : CodeParameters.PASSTHROUGH_DISENGAGED_POS);
    }

    public void intake() {
        intakeMotor.set(CodeParameters.INTAKE_POWER);
        state = IntakeState.INTAKE;
    }

    public void reverse() {
        intakeMotor.set(-CodeParameters.INTAKE_POWER);
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

    public boolean intakeJustGotFull() {
        return intakeGotFull.isJustActive();
    }
}
