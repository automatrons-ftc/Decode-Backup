package org.firstinspires.ftc.teamcode.Autos;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.DecodeRobot;
import org.firstinspires.ftc.teamcode.Mechanisms.CommandSeriesVault;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.FollowerCommand;

@Autonomous(name = "RED_Preloads", group = "Autonomous")
@Configurable
public class RED_Preloads extends CommandOpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private RobotMap robotMap;

    private Intake intake;
    private Shooter shooter;

    private CommandSeriesVault commandVault;

    private Timer loopTime, elapsedTime;

    private Paths paths;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        robotMap = new RobotMap(hardwareMap, telemetry,null,null);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(80, 8, Math.toRadians(90)));
        paths = new Paths(follower);

        intake = new Intake(robotMap);
        shooter = new Shooter(robotMap, () -> follower.getPose(), DecodeRobot.Alliance.RED, telemetry);
        commandVault = new CommandSeriesVault(intake, shooter);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        loopTime = new Timer();
        elapsedTime = new Timer();

        new SequentialCommandGroup(
                new InstantCommand(() -> elapsedTime.resetTimer()),
                new FollowerCommand(follower, paths.StartToShootPreload, 0.7),
                commandVault.shootingProc(),
                new FollowerCommand(follower, paths.ShootPreloadToPark, 0.7)
        ).schedule();
    }

    @Override
    public void run() {
        super.run();
        follower.update();

        for (LynxModule hub : robotMap.getHubs()) hub.clearBulkCache();

        telemetry.addData("Loop Hz: ", 1.0/loopTime.getElapsedTimeSeconds());
        loopTime.resetTimer();
        telemetry.update();
    }


    public static class Paths {

        private final double deccel_strength = 0.5;
        public PathChain
                StartToShootPreload,
                ShootPreloadToPark;

        public Paths(Follower follower) {
            StartToShootPreload = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(80, 8),
                                    new Pose(80, 24)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(90))
                    .setBrakingStrength(deccel_strength)
                    .build();

            ShootPreloadToPark = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(80, 24),
                                    new Pose(133, 7.5)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(90))
                    .setBrakingStrength(deccel_strength)
                    .build();
        }
    }
}