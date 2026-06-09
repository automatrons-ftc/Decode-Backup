package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.Hardware.GamepadExEx;
import org.firstinspires.ftc.teamcode.Hardware.IMUSubsystem;
import org.firstinspires.ftc.teamcode.Mechanisms.IndicatorSubsystem;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.MathFunction;

public class DecodeRobot {
    public enum Alliance {
        RED,
        BLUE
    }
    protected Alliance alliance;

    protected FtcDashboard dashboard;
    protected GamepadExEx driverOp, toolOp;
    protected Telemetry telemetry;

    protected MecanumDrive drive = null;
    protected IMUSubsystem gyro;
    protected IMU imu;

    private boolean hasInit = false;

    protected PinpointLocalizer teleOpLocalizer;

    // Mechanisms
    protected Intake intake;
    protected Shooter shooter;
    protected IndicatorSubsystem indicator;

    protected MotifStorage.MotifState motif;

    private double yawDriverRelativeOffset = 0;

    public DecodeRobot(RobotMap robotMap, DriveConstants driveConstants, Alliance alliance,
                       Pose pose, MotifStorage.MotifState motif
    ) {
        this.alliance = alliance;
        this.motif = motif;

        initCommon(robotMap, driveConstants);
        initTele(robotMap, pose);

        // Init Mechanisms when driver starts moving the robot
        this.initMechanismsTeleOp(robotMap);
//        new Trigger(() -> (Math.abs(drivetrainForward()) > 0.1 ||
//            Math.abs(drivetrainStrafe()) > 0.1 ||
//            Math.abs(drivetrainTurn()) > 0.1) && !hasInit)
//            .whenActive(new InstantCommand(() -> this.initMechanismsTeleOp(robotMap)));
    }

    public DecodeRobot(RobotMap robotMap, DriveConstants driveConstants, Alliance alliance
    ) {
        this.alliance = alliance;

        initCommon(robotMap, driveConstants);
        initAuto();
    }

    /*-- Drive Commands --*/
    public double drivetrainStrafe() {
        return driverOp.getLeftX();
    }

    public double drivetrainForward() {
        return driverOp.getLeftY();
    }

    public double drivetrainTurn() {
        return driverOp.getRightX();
    }

    public void drive_update() {
        teleOpLocalizer.update();

        telemetry.addData("Pose", "X: %.2f, Y: %.2f, Theta: %.2f",
            getPose().getX(), getPose().getY(), Math.toDegrees(getPose().getHeading()));

        telemetry.addData("Field Centric Yaw: ", "X: %.2f", getFieldCentricHeading());

        drive.drive(
            drivetrainStrafe(),
            drivetrainForward(),
            drivetrainTurn(),
    Math.toDegrees(getPose().getHeading())-(getAlliance() == Alliance.RED ? 0 : 180)-yawDriverRelativeOffset,
//            getFieldCentricHeading(),
            driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)
        );
    }

    public void drive_update(Pose pose) {
        drive.drive(
            pose.getX(),
            pose.getY(),
                getPose().getHeading(),
            0,
            0
        );
    }

    /*-- Drive Type Pick --*/
    public void setFieldCentric() {
        drive.setFieldCentric();
    }

    public void setRobotCentric() {
        drive.setRobotCentric();
    }

    public void resetFieldCentricReference() {
        yawDriverRelativeOffset = getHeading()-(getAlliance() == Alliance.RED ? 0 : 180);
    }

    public void setAutoEnabled(boolean enabled) {
        drive.setAutoEnabled(enabled);
    }

    /*-- Getters --*/
    public double getHeading() {
        return gyro.getRawYaw();
    }

    public double getFieldCentricHeading() {
        return gyro.getFieldCentricYaw();
    }

    public double getContinuousHeading() {
        return gyro.getYaw();
    }

    public double getHeadingVelocity() {
        return 0.0; // TODO: Implement
    }

    public Alliance getAlliance() {
        return alliance;
    }
    public MotifStorage.MotifState getMotif() {
        return motif;
    }

    public Pose getPose() {
        return teleOpLocalizer.getPose();
    }
    public Pose getPoseVelocity() {
        return teleOpLocalizer.getVelocity();
    }

    /*-- Initializations --*/
    public void initCommon(RobotMap robotMap, DriveConstants driveConstants) {
        //- Camera
        this.dashboard = FtcDashboard.getInstance();

        //- Telemetries
        this.telemetry = robotMap.getTelemetry();

        //- Drive
        drive = new MecanumDrive(robotMap, driveConstants);
    }

    public void initAuto() {
        //- Setup and Initialize Mechanisms Objects
        initMechanismsAutonomous();
    }

    public void initTele(RobotMap robotMap, Pose startingPose) {
        teleOpLocalizer = new PinpointLocalizer(robotMap.getHm(), Constants.localizerConstants, startingPose);

        gyro = new IMUSubsystem(
            robotMap, () -> (MathFunction.wrapDegrees(Math.toDegrees(getPose().getHeading())))
        );

        CommandScheduler.getInstance().registerSubsystem(gyro);

        //- Gamepads
        this.driverOp = robotMap.getDriverOp();
        this.toolOp = robotMap.getToolOp();
    }

    /*-- Mechanisms Initialization --*/
    public void initMechanismsAutonomous() {
        //TODO: make init Mechanisms
    }

    public void initMechanismsTeleOp(RobotMap robotMap) {
        hasInit = true;
        intake = new Intake(robotMap);
        shooter = new Shooter(robotMap, this::getPose, this::getPoseVelocity,
                () -> new Pose(0, 0, 0), alliance,
                true, false, false, false);
        indicator = new IndicatorSubsystem(robotMap);


        driverOp.getGamepadButton(GamepadKeys.Button.START).whenPressed(this::resetFieldCentricReference);

        toolOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new ConditionalCommand(
                new SequentialCommandGroup(
                        new InstantCommand(intake::intake),
                        new InstantCommand(() -> indicator.setState(IndicatorSubsystem.IndicatorState.INTAKING))
                ),
                new SequentialCommandGroup(
                        new InstantCommand(intake::stop),
                        new InstantCommand(() -> indicator.setState(IndicatorSubsystem.IndicatorState.IDLE))
                ),
                () -> intake.getState() == Intake.IntakeState.STOPPED
        ));

        new Trigger(intake::intakeJustGotFull).whenActive(new SequentialCommandGroup(
                new InstantCommand(intake::stop),
                new InstantCommand(() -> indicator.setState(IndicatorSubsystem.IndicatorState.FULL))
        ));

        toolOp.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new ConditionalCommand(
                new InstantCommand(intake::engagePassthough),
                new InstantCommand(intake::disengagePassthough),
                () -> !intake.isPassthoughEngaged()
        ));

        toolOp.getGamepadButton(GamepadKeys.Button.X).whenPressed(new ConditionalCommand(
                new InstantCommand(shooter::closeFinger),
                new InstantCommand(shooter::openFinger),
                () -> shooter.isFingerOpen()
        ));

        toolOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(intake::disengagePassthough),
                                new InstantCommand(intake::intake),
                                new WaitUntilCommand(shooter::wheelsAtSpeed),
                                new InstantCommand(shooter::openFinger),
                                new InstantCommand(() -> indicator.setState(IndicatorSubsystem.IndicatorState.IDLE)),
                                new WaitCommand(200),
                                new InstantCommand(intake::engagePassthough),
                                new WaitCommand(1000),
                                new InstantCommand(intake::disengagePassthough),
                                new InstantCommand(shooter::closeFinger),
                                new InstantCommand(() -> indicator.setState(IndicatorSubsystem.IndicatorState.INTAKING))
                        ),
                        new InstantCommand(),
                        () -> shooter.turretInRange() && shooter.inLUTRange() && shooter.areWheelsEnabled()
                )
        );

        toolOp.getGamepadButton(GamepadKeys.Button.A).whenPressed(new ConditionalCommand(
                new InstantCommand(shooter::disableWheels),
                new InstantCommand(shooter::enableWheels),
                shooter::areWheelsEnabled
        ));

        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new InstantCommand(shooter::decrease_turret_offset)
        );

        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new InstantCommand(shooter::increase_turret_offset)
        );
    }
}
