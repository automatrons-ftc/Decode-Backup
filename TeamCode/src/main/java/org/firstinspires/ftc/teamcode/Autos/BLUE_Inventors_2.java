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
import com.pedropathing.geometry.BezierCurve;
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

@Autonomous(name = "BLUE_Inventors_2", group = "Autonomous")
@Configurable
public class BLUE_Inventors_2 extends CommandOpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private RobotMap robotMap;

    private Intake intake;
    private Shooter shooter;

    private CommandSeriesVault commandVault;

//    private Timer loopTime;
    private Timer elapsedTime;

    private Paths paths;

    boolean first = true;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        robotMap = new RobotMap(hardwareMap, telemetry,null,null);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(144-80, 8, Math.toRadians(90)));
        paths = new Paths(follower);

        intake = new Intake(robotMap);
        shooter = new Shooter(robotMap, () -> follower.getPose(), DecodeRobot.Alliance.BLUE, telemetry);
        commandVault = new CommandSeriesVault(intake, shooter);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

//        loopTime = new Timer();
        elapsedTime = new Timer();

        new SequentialCommandGroup(
//                new InstantCommand(this::initTime),
                new FollowerCommand(follower, paths.StartToShootPreload, 0.95),
                commandVault.shootingProc(),
                new InstantCommand(intake::intake),
                new FollowerCommand(follower, paths.ShootPreloadIntakeGate, 0.9),
                new WaitUntilCommand(() -> elapsedTime.getElapsedTimeSeconds() > 10.8),
                new WaitCommand(400),
                new InstantCommand(intake::stop),
                new FollowerCommand(follower, paths.IntakeGateToShoot, 1),
                commandVault.shootingProc(),
                new InstantCommand(intake::intake),
                new FollowerCommand(follower, paths.ShootToIntakeHP, 1),
                new WaitUntilCommand(() -> elapsedTime.getElapsedTimeSeconds() > 20.7),
                new InstantCommand(intake::stop),
                new FollowerCommand(follower, paths.IntakeHPToShoot, 1),
                commandVault.shootingProc(),
                new FollowerCommand(follower, paths.ShootToPark, 1)
        ).schedule();
    }

    public void initTime() {
        elapsedTime.resetTimer();
    }



    @Override
    public void run() {
        if(first){
            first = false;
            elapsedTime.resetTimer();
        }

        super.run();
        follower.update();

        for (LynxModule hub : robotMap.getHubs()) hub.clearBulkCache();

//        telemetry.addData("Loop Hz: ", 1.0/loopTime.getElapsedTimeSeconds());
        telemetry.addData("Timer: ", elapsedTime.getElapsedTimeSeconds());
        telemetry.addData("Pose", "X: %.2f, Y: %.2f, Theta: %.2f",
                follower.getPose().getX(), follower.getPose().getY(), Math.toDegrees(follower.getPose().getHeading()));
//        loopTime.resetTimer();
        telemetry.update();
    }


    public static class Paths {

        private final double deccel_strength = 0.5;
        public PathChain
                StartToShootPreload,
                ShootPreloadIntakeGate,
                IntakeGateToShoot,
                ShootToIntakeHP,
                IntakeHPToShoot,
                ShootToPark;

        public Paths(Follower follower) {
            StartToShootPreload = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(144-80, 8),
                                    new Pose(144-83, 22)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(120))
                    .setBrakingStart(0.4)
                    .setBrakingStrength(0.2)
                    .build();

            ShootPreloadIntakeGate = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(144-80, 24),
                                    new Pose(144-132, 6),
                                    new Pose(144-132, 24)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(120))
                    .setBrakingStrength(0.6)
                    .build();

            IntakeGateToShoot = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(144-133.3, 24),
                                    new Pose(144-132, 6),
                                    new Pose(144-85, 22)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(120))
                    .setBrakingStrength(1)
                    .build();

            ShootToIntakeHP = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(144-80, 24.0),
                                    new Pose(144-110.0, 5.0),
                                    new Pose(144-130, 8)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(180))
                    .setBrakingStrength(deccel_strength)
                    .build();

            IntakeHPToShoot = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(144-130, 8),
                                    new Pose(144-140.0, 50.0),
                                    new Pose(144-86, 76)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(160))
                    .setBrakingStrength(deccel_strength)
                    .build();

            ShootToPark = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(144-86, 76),
                                    new Pose(144-95, 74)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(180))
                    .setBrakingStrength(deccel_strength)
                    .build();
        }
    }
}