package org.firstinspires.ftc.teamcode.Autos;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
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
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.NanoTimer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CodeParameters;
import org.firstinspires.ftc.teamcode.DecodeRobot;
import org.firstinspires.ftc.teamcode.Mechanisms.CommandSeriesVault;
import org.firstinspires.ftc.teamcode.Mechanisms.IndicatorSubsystem;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.FollowerCommand;

@Autonomous(name = "RED_FAR", group = "Autonomous")
@Configurable
public class RED_FAR extends CommandOpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private RobotMap robotMap;

    private Intake intake;
    private Shooter shooter;
    private IndicatorSubsystem indicator;

    private CommandSeriesVault commandVault;

    private Timer loopTime;

    private Paths paths;
    private Pose pinpointPose;
    private NanoTimer timer;
    private long deltaTimeNano;

    @Override
    public void initialize() {
        pinpointPose = CodeParameters.RED_START_POSE_FAR;
        timer = new NanoTimer();
        deltaTimeNano = 1;
        CommandScheduler.getInstance().reset(); // Ultra SOS
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        robotMap = new RobotMap(hardwareMap, telemetry,null,null);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(CodeParameters.RED_START_POSE_FAR);
        paths = new Paths(follower);

        intake = new Intake(robotMap);
        shooter = new Shooter(robotMap, this::getPose, this::getVelPose, this::getAccelPose,
                DecodeRobot.Alliance.RED, false, false, true);
        indicator = new IndicatorSubsystem(robotMap);
        commandVault = new CommandSeriesVault(intake, shooter, indicator);

        commandVault.enableWheels().schedule();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        loopTime = new Timer();

        new SequentialCommandGroup(
                new FollowerCommand(follower, paths.StartToShootPreloads, 1, false),
                commandVault.shootingProcAutoFAR(),
                commandVault.startIntake(),
                new FollowerCommand(follower, paths.ShootPreloadsToStack3, 1, false),
                commandVault.stopIntake(),
                new FollowerCommand(follower, paths.Stack3ToShoot, 1, false),
                commandVault.shootingProcAutoFAR(),
                commandVault.startIntake(),
                new FollowerCommand(follower, paths.ShootToHP, 1, false),
                new WaitCommand(400),
                new FollowerCommand(follower, paths.HPToShoot, 1, false),
                commandVault.shootingProcAutoFAR(),

                // Repeat
                commandVault.startIntake(),
                new FollowerCommand(follower, paths.ShootToHP, 1, false),
                new WaitCommand(300),
                commandVault.stopIntake(),
                new FollowerCommand(follower, paths.HPToShoot, 1, false),
                commandVault.shootingProcAutoFAR(),

                // Repeat
                commandVault.startIntake(),
                new FollowerCommand(follower, paths.ShootToHP, 1, false),
                new WaitCommand(300),
                commandVault.stopIntake(),
                new FollowerCommand(follower, paths.HPToShoot, 1, false),
                commandVault.shootingProcAutoFAR(),

                // Repeat
                commandVault.startIntake(),
                new FollowerCommand(follower, paths.ShootToHP, 1, false),
                new WaitCommand(300),
                commandVault.stopIntake(),
                new FollowerCommand(follower, paths.HPToShoot, 1, false),
                commandVault.shootingProcAutoFAR(),

                // Repeat
                commandVault.startIntake(),
                new FollowerCommand(follower, paths.ShootToHP, 1, false),
                new WaitCommand(300),
                commandVault.stopIntake(),
                new FollowerCommand(follower, paths.HPToShoot, 1, false),
                commandVault.shootingProcAutoFAR(),

                new FollowerCommand(follower, paths.ShootToPark, 1, false)

        ).schedule();
    }

    @Override
    public void run() {
        super.run();
        follower.update();

        for (LynxModule hub : robotMap.getHubs()) hub.clearBulkCache();

        telemetry.addData("Loop Hz: ", 1.0/loopTime.getElapsedTimeSeconds());
        loopTime.resetTimer();

        telemetry.addData("X", getPose().getX());
        telemetry.addData("Y", getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(getPose().getHeading()));
        telemetry.update();
    }

    public static class Paths {
        private final double deccel_strength = 0;

        public PathChain
                StartToShootPreloads,
                ShootPreloadsToStack3,
                Stack3ToShoot,
                ShootToHP,
                HPToShoot,
                ShootToPark;

        public Paths(Follower follower) {
            StartToShootPreloads = follower.pathBuilder().addPath(
                            new BezierLine(
                                    CodeParameters.RED_START_POSE_FAR,
                                    CodeParameters.RED_SHOOT_PRELOADS_POSE_FAR
                            )
                    ).setLinearHeadingInterpolation(CodeParameters.RED_START_POSE_FAR.getHeading(), CodeParameters.RED_SHOOT_PRELOADS_POSE_FAR.getHeading())
                    .build();

            ShootPreloadsToStack3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    CodeParameters.RED_SHOOT_PRELOADS_POSE_FAR,
                                    new Pose(96, 40),
                                    CodeParameters.RED_STACK3_POSE
                            )).setHeadingInterpolation(HeadingInterpolator.piecewise(
                                    new HeadingInterpolator.PiecewiseNode(
                                            0,
                                            .4,
                                            HeadingInterpolator.linear(CodeParameters.RED_SHOOT_PRELOADS_POSE_FAR.getHeading(), CodeParameters.RED_STACK3_POSE.getHeading())
                                    ),
                                    new HeadingInterpolator.PiecewiseNode(
                                            .4,
                                            1,
                                            HeadingInterpolator.constant(CodeParameters.RED_STACK3_POSE.getHeading())
                                    )
                            )).build();

            Stack3ToShoot = follower.pathBuilder().addPath(
                    new BezierLine(
                            CodeParameters.RED_STACK3_POSE,
                            CodeParameters.RED_SHOOT_POSE_FAR
                    )).setConstantHeadingInterpolation(CodeParameters.RED_SHOOT_POSE_FAR.getHeading())
                    .build();

            ShootToHP = follower.pathBuilder().addPath(
                    new BezierCurve(
                            CodeParameters.RED_SHOOT_POSE_FAR,
                            new Pose(98, 11),
                            CodeParameters.RED_HP_POSE
                    )).setConstantHeadingInterpolation(CodeParameters.RED_HP_POSE.getHeading())
                    .build();

            HPToShoot = follower.pathBuilder().addPath(
                    new BezierCurve(
                            CodeParameters.RED_HP_POSE,
                            new Pose(98, 11),
                            CodeParameters.RED_SHOOT_POSE_FAR
                    )).setConstantHeadingInterpolation(CodeParameters.RED_SHOOT_POSE_FAR.getHeading())
                    .build();

            ShootToPark = follower.pathBuilder().addPath(
                    new BezierLine(
                            CodeParameters.RED_SHOOT_POSE_FAR,
                            CodeParameters.RED_PARK_POSE_FAR
                    )).setConstantHeadingInterpolation(CodeParameters.RED_PARK_POSE_FAR.getHeading())
                    .build();
        }
    }

    public Pose getPose() {
        return follower.getPose();
    }

    public Pose getVelPose() {
        deltaTimeNano = timer.getElapsedTime();
        timer.resetTimer();

        Pose deltaPose = follower.getPose().minus(pinpointPose);
        pinpointPose = follower.getPose();

        return new Pose(
            deltaPose.getX() / (deltaTimeNano / Math.pow(10.0, 9)),
            deltaPose.getY() / (deltaTimeNano / Math.pow(10.0, 9)),
            0
        );
    }

    public Pose getAccelPose() {

        return new Pose(
            follower.getAcceleration().getXComponent(),
            follower.getAcceleration().getYComponent(),
            0
        );
    }

    @Override
    public void reset() {
        super.reset();
        PoseStorage.currentPose = getPose();
    }
}