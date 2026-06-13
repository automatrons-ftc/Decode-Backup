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
        telemetry.addData("[Indicator] Timer ", stateTimer.getElapsedTimeSeconds());

        if(state != previousState) stateTimer.resetTimer();

//        switch (state) {
//            case INTAKING:
//                if (stateTimer.getElapsedTimeSeconds() % 1.0 < 0.5) {
//                    indicator.setPosition(CodeParameters.RED_COLOR);
//                } else {
//                    indicator.setPosition(CodeParameters.BLACK_COLOR);
//                }
//                break;
//            case FULL:
//                if (stateTimer.getElapsedTimeSeconds() % 1.0 < 0.5) {
//                    indicator.setPosition(CodeParameters.GREEN_COLOR);
//                } else {
//                    indicator.setPosition(CodeParameters.BLACK_COLOR);
//                }
//                break;
//            case IDLE:
//                indicator.setPosition(CodeParameters.BLACK_COLOR);
//                break;
//        }

//        switch (state) {
//            case INTAKING:
//                if (stateTimer.getElapsedTimeSeconds() % 0.5 < 0.25) {
//                    indicator.setPosition(0.3);
//                } else {
//                    indicator.setPosition(0.0);
//                }
//                break;
//            case FULL:
//                if (stateTimer.getElapsedTimeSeconds() % 0.5 < 0.25) {
//                    indicator.setPosition(0.5);
//                } else {
//                    indicator.setPosition(0.0);
//                }
//                break;
//            case IDLE:
//                indicator.setPosition(0.0);
//                break;
//        }

        switch (state) {
            case INTAKING:
                indicator.setPosition(CodeParameters.RED_COLOR);
                break;
            case FULL:
                indicator.setPosition(CodeParameters.GREEN_COLOR);
                break;
            case IDLE:
                indicator.setPosition(CodeParameters.BLACK_COLOR);
                break;
        }

        previousState = state;
    }

    public void setState(IndicatorState newState) {
        state = newState;
    }

    public IndicatorState getState() {
        return state;
    }
}
