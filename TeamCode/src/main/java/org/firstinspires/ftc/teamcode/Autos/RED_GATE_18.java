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

@Autonomous(name = "RED_GATE_18", group = "Autonomous")
@Configurable
public class RED_GATE_18 extends CommandOpMode {
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
        pinpointPose = CodeParameters.RED_START_POSE;
        timer = new NanoTimer();
        deltaTimeNano = 1;
        CommandScheduler.getInstance().reset(); // Ultra SOS
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        robotMap = new RobotMap(hardwareMap, telemetry,null,null);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(CodeParameters.RED_START_POSE);
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
                new FollowerCommand(follower, paths.StartToShootPreload, 1, false),
                commandVault.shootingProcAuto(),
                commandVault.startIntake(),
                new FollowerCommand(follower, paths.ShootPreloadToStack2, 1, false),
                commandVault.stopIntake(),
                new ParallelCommandGroup(
                        new FollowerCommand(follower, paths.Stack2ToShoot,1, false),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> follower.atPose(CodeParameters.RED_SHOOT_POSE, CodeParameters.SHOOTING_TOLERANCE, CodeParameters.SHOOTING_TOLERANCE)),
                                commandVault.shootingProcAuto()
                        )
                ),
                // ------------------------------------- 1 -------------------------------------- //
                commandVault.startIntake(),
                new FollowerCommand(follower, paths.ShootToGate, 1, false),
                commandVault.gateWaitCmd(),
                commandVault.stopIntake(),
                new ParallelCommandGroup(
                        new FollowerCommand(follower, paths.GateToShoot, 1, false),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> follower.atPose(CodeParameters.RED_SHOOT_POSE, CodeParameters.SHOOTING_TOLERANCE, CodeParameters.SHOOTING_TOLERANCE)),
                                commandVault.shootingProcAuto()
                        )
                ),
                // ------------------------------------- 2 -------------------------------------- //
                commandVault.startIntake(),
                new FollowerCommand(follower, paths.ShootToGate, 1, false),
                commandVault.gateWaitCmd(),
                commandVault.stopIntake(),
                new ParallelCommandGroup(
                        new FollowerCommand(follower, paths.GateToShoot, 1, false),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> follower.atPose(CodeParameters.RED_SHOOT_POSE, CodeParameters.SHOOTING_TOLERANCE, CodeParameters.SHOOTING_TOLERANCE)),
                                commandVault.shootingProcAuto()
                        )
                ),
                // ------------------------------------- 3 -------------------------------------- //
                commandVault.startIntake(),
                new FollowerCommand(follower, paths.ShootToGate, 1, false),
                commandVault.gateWaitCmd(),
                commandVault.stopIntake(),
                new ParallelCommandGroup(
                        new FollowerCommand(follower, paths.GateToShoot, 1, false),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> follower.atPose(CodeParameters.RED_SHOOT_POSE, CodeParameters.SHOOTING_TOLERANCE, CodeParameters.SHOOTING_TOLERANCE)),
                                commandVault.shootingProcAuto()
                        )
                ),
                // ---------------------------------- STACK 1 ----------------------------------- //
                commandVault.startIntake(),
                new FollowerCommand(follower, paths.ShootToStack1, 1, false),
                commandVault.stopIntake(),
                new FollowerCommand(follower, paths.Stack1ToPark, 1, false),
                commandVault.shootingProcAuto()
                //park shooter
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
                StartToShootPreload,
                ShootPreloadToStack2,
                Stack2ToShoot,
                ShootToGate,
                GateToShoot,
                ShootToStack1,
                Stack1ToPark;

        public Paths(Follower follower) {
            StartToShootPreload = follower.pathBuilder().addPath(
                            new BezierLine(
                                    CodeParameters.RED_START_POSE,
                                    CodeParameters.RED_SHOOT_PRELOADS_POSE
                            )
                    ).setConstantHeadingInterpolation(CodeParameters.RED_START_POSE.getHeading())
                    .build();

            ShootPreloadToStack2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    CodeParameters.RED_SHOOT_PRELOADS_POSE,
                                    new Pose(94.4, 57.9),
                                    CodeParameters.RED_STACK2_POSE
                            )
                    ).setHeadingInterpolation(HeadingInterpolator.piecewise(
                            new HeadingInterpolator.PiecewiseNode(
                                    0,
                                    .2,
                                    HeadingInterpolator.linear(CodeParameters.RED_SHOOT_PRELOADS_POSE.getHeading(), CodeParameters.RED_STACK2_POSE.getHeading())
                            ),
                            new HeadingInterpolator.PiecewiseNode(
                                    .2,
                                    1,
                                    HeadingInterpolator.constant(CodeParameters.RED_STACK2_POSE.getHeading())
                            )
                    ))
                    .build();

            Stack2ToShoot = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    CodeParameters.RED_STACK2_POSE,
                                    new Pose(99, 58),
                                    CodeParameters.RED_SHOOT_POSE
                            )
                    ).setHeadingInterpolation(HeadingInterpolator.piecewise(
                            new HeadingInterpolator.PiecewiseNode(
                                    0,
                                    .2,
                                    HeadingInterpolator.linear(CodeParameters.RED_STACK2_POSE.getHeading(), CodeParameters.RED_SHOOT_POSE.getHeading())
                            ),
                            new HeadingInterpolator.PiecewiseNode(
                                    .2,
                                    1,
                                    HeadingInterpolator.constant(CodeParameters.RED_SHOOT_POSE.getHeading()
                            )
                    )))
                    .build();

            ShootToGate = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    CodeParameters.RED_SHOOT_POSE,
                                    new Pose(92, 60),
                                    CodeParameters.RED_GATE_POSE
                            )
                    ).setHeadingInterpolation(HeadingInterpolator.piecewise(
                            new HeadingInterpolator.PiecewiseNode(
                                    0,
                                    .8,
                                    HeadingInterpolator.linear(CodeParameters.RED_SHOOT_POSE.getHeading(), CodeParameters.RED_GATE_POSE.getHeading())
                            ),
                            new HeadingInterpolator.PiecewiseNode(
                                    .8,
                                    1,
                                    HeadingInterpolator.constant(CodeParameters.RED_GATE_POSE.getHeading())
                            )
                    ))
                    .build();

            GateToShoot = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    CodeParameters.RED_GATE_POSE,
                                    new Pose(92, 60),
                                    CodeParameters.RED_SHOOT_POSE
                            )
                    ).setHeadingInterpolation(HeadingInterpolator.piecewise(
                            new HeadingInterpolator.PiecewiseNode(
                                    0,
                                    .1,
                                    HeadingInterpolator.linear(CodeParameters.RED_GATE_POSE.getHeading(), 0)
                            ),
                            new HeadingInterpolator.PiecewiseNode(
                                    .1,
                                    .15,
                                    HeadingInterpolator.constant(0)
                            ),
                            new HeadingInterpolator.PiecewiseNode(
                                    .15,
                                    .3,
                                    HeadingInterpolator.linear(0, CodeParameters.RED_SHOOT_POSE.getHeading())
                            ),
                            new HeadingInterpolator.PiecewiseNode(
                                    .3,
                                    1,
                                    HeadingInterpolator.constant(CodeParameters.RED_SHOOT_POSE.getHeading())
                            )
                    ))
                    .build();

            ShootToStack1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    CodeParameters.RED_SHOOT_POSE,
                                    CodeParameters.RED_STACK1_POSE
                            )
                    ).setHeadingInterpolation(HeadingInterpolator.piecewise(
                            new HeadingInterpolator.PiecewiseNode(
                                    0,
                                    .2,
                                    HeadingInterpolator.linear(CodeParameters.RED_SHOOT_POSE.getHeading(), CodeParameters.RED_STACK1_POSE.getHeading())
                            ),
                            new HeadingInterpolator.PiecewiseNode(
                                    .2,
                                    1,
                                    HeadingInterpolator.constant(CodeParameters.RED_STACK1_POSE.getHeading())
                            )
                    ))
                    .build();

            Stack1ToPark = follower.pathBuilder().addPath(
                            new BezierLine(
                                    CodeParameters.RED_STACK1_POSE,
                                    CodeParameters.RED_PARK_POSE
                            )
                    ).setHeadingInterpolation(HeadingInterpolator.piecewise(
                            new HeadingInterpolator.PiecewiseNode(
                                    0,
                                    .2,
                                    HeadingInterpolator.linear(CodeParameters.RED_STACK1_POSE.getHeading(), CodeParameters.RED_PARK_POSE.getHeading())
                            ),
                            new HeadingInterpolator.PiecewiseNode(
                                    .2,
                                    1,
                                    HeadingInterpolator.constant(CodeParameters.RED_PARK_POSE.getHeading())
                            )
                    ))
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