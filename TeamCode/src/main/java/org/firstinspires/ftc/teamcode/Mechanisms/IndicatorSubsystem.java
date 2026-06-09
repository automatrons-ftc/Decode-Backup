package org.firstinspires.ftc.teamcode.Mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CodeParameters;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.Util.Timer;

@Config
public class IndicatorSubsystem extends SubsystemBase {
    private final ServoImplEx indicator;

    public enum IndicatorState {
        IDLE,
        INTAKING,
        FULL
    }

    private IndicatorState state = IndicatorState.IDLE, previousState = IndicatorState.IDLE;
    private Timer stateTimer = new Timer();
    private Telemetry telemetry;

    public IndicatorSubsystem(RobotMap robotMap) {
        this.indicator = robotMap.getIndicator();
        this.telemetry = robotMap.getTelemetry();
    }

    @Override
    public void periodic() {
        telemetry.addData("[Indicator] State ", state);

        if(state != previousState) stateTimer.resetTimer();

        switch (state) {
            case INTAKING:
                if (stateTimer.getElapsedTimeSeconds() % 2.0*(1.0/CodeParameters.BLINKING_FREQUENCY) < (1.0/CodeParameters.BLINKING_FREQUENCY)) {
                    indicator.setPosition(CodeParameters.RED_COLOR);
                } else {
                    indicator.setPosition(CodeParameters.BLACK_COLOR);
                }
                break;
            case FULL:
                if (stateTimer.getElapsedTimeSeconds() % 2.0*(1.0/CodeParameters.BLINKING_FREQUENCY) < (1.0/CodeParameters.BLINKING_FREQUENCY)) {
                    indicator.setPosition(CodeParameters.GREEN_COLOR);
                } else {
                    indicator.setPosition(CodeParameters.BLACK_COLOR);
                }
                break;
            case IDLE:
                indicator.setPosition(CodeParameters.BLACK_COLOR);
                break;
        }
    }

    public void setState(IndicatorState newState) {
        previousState = state;
        state = newState;
    }

    public IndicatorState getState() {
        return state;
    }
}
