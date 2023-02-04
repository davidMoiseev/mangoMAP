package frc.robot.subsystems;

import frc.robot.RobotState;
import frc.robot.RobotCommander;

public abstract class SubsystemBase {
    public abstract void enabledAction(RobotState robotState, RobotCommander commander);

    public abstract void disabledAction(RobotState robotState);

    public abstract void updateState();

    public abstract void zeroActuators();

    public abstract void zeroSensor();
    
    public abstract void logData();
}
