package org.firstinspires.ftc.teamcode.Mechanisms;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.DecodeRobot;

public class CommandSeriesVault {
    private DecodeRobot.Alliance alliance;
    private Intake intake;
    private Shooter shooter;

    // --------------------------------------- Constants ---------------------------------------- //
    public static int FINGER_BETWEEN_MS = 80, FINGER_HOLD_MS = 340;

    public CommandSeriesVault(Intake intake, Shooter shooter) {
        this.intake = intake;
        this.shooter = shooter;
    }

    public InstantCommand enableWheels() {
        return new InstantCommand(shooter::enableWheels, shooter);
    }

    public InstantCommand disableWheels() {
        return new InstantCommand(shooter::disableWheels, shooter);
    }

    public InstantCommand reverseIntake() {
        return new InstantCommand(intake::reverse, intake);
    }

    public InstantCommand stopIntake() {
        return new InstantCommand(intake::stop, intake);
    }


    public SequentialCommandGroup shootingProc() {
        return new SequentialCommandGroup(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(intake::disengagePassthough),
                                new InstantCommand(shooter::enableWheels),
                                new WaitUntilCommand(shooter::turretAtTarget),
                                new WaitUntilCommand(shooter::wheelsAtSpeed),
                                new InstantCommand(intake::intake),
                                new WaitCommand(200),
                                new InstantCommand(intake::engagePassthough),
                                new WaitCommand(1500),
                                new InstantCommand(intake::disengagePassthough),
                                new InstantCommand(shooter::disableWheels)
                        ),
                        new InstantCommand(),
                        () -> shooter.turretInRange() && shooter.inLUTRange()
                )
        );
    }
}
