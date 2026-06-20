package org.firstinspires.ftc.teamcode.Mechanisms;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.CodeParameters;
import org.firstinspires.ftc.teamcode.DecodeRobot;

public class CommandSeriesVault {
    private DecodeRobot.Alliance alliance;
    private Intake intake;
    private Shooter shooter;
    private IndicatorSubsystem indicator;

    // --------------------------------------- Constants ---------------------------------------- //
    public static int FINGER_BETWEEN_MS = 80, FINGER_HOLD_MS = 340;

    public CommandSeriesVault(Intake intake, Shooter shooter, IndicatorSubsystem indicator) {
        this.intake = intake;
        this.shooter = shooter;
        this.indicator = indicator;

        new Trigger(intake::intakeJustGotFull).whenActive(new InstantCommand(intake::stop));
    }

    public InstantCommand enableWheels() {
        return new InstantCommand(shooter::enableWheels, shooter);
    }

    public InstantCommand disableWheels() {
        return new InstantCommand(shooter::disableWheels, shooter);
    }

    public InstantCommand startIntake() {
        return new InstantCommand(intake::intake);
    }

    public InstantCommand stopIntake() {
        return new InstantCommand(intake::stop);
    }

    public InstantCommand reverseIntake() {
        return new InstantCommand(intake::reverse, intake);
    }

    public SequentialCommandGroup shootingProcAuto() {
        return new SequentialCommandGroup(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(intake::intake),
                                new InstantCommand(shooter::openFinger),
                                new WaitCommand(80),
                                new InstantCommand(intake::engagePassthough),
                                new WaitCommand(500),
                                new InstantCommand(intake::stop),
                                new InstantCommand(intake::disengagePassthough),
                                new InstantCommand(shooter::closeFinger)
                        ),
                        new InstantCommand(),
                        () -> shooter.turretInRange() && shooter.inLUTRange()
                )
        );
    }

    public SequentialCommandGroup shootingProcAutoFAR() {
        return new SequentialCommandGroup(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> shooter.turretAtGoal() && shooter.wheelsAtSpeed()),
                                new InstantCommand(intake::intake),
                                new InstantCommand(shooter::openFinger),
                                new WaitCommand(80),
                                new InstantCommand(intake::engagePassthough),
                                new WaitCommand(500),
                                new InstantCommand(intake::stop),
                                new InstantCommand(intake::disengagePassthough),
                                new InstantCommand(shooter::closeFinger)
                        ),
                        new InstantCommand(),
                        () -> shooter.turretInRange() && shooter.inLUTRange()
                )
        );
    }

    public InstantCommand setAutoAngle(double angle) {
        return new InstantCommand(() -> shooter.enableAutoCustom(angle), shooter);
    }

    public InstantCommand parkShooter() {
        return new InstantCommand(shooter::enableParkingState, shooter);
    }

//    public WaitCommand gateWaitCmd() { // TODO: Use Sensors
//        return new WaitCommand(CodeParameters.GATE_WAIT_TIME);
//    }

    public ParallelRaceGroup gateWaitCmd() {
        return new ParallelRaceGroup(
                new WaitCommand(CodeParameters.GATE_WAIT_TIME),
                new WaitUntilCommand(intake::intakeJustGotFull)
        );
    }
}
