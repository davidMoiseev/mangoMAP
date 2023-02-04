package frc.robot.sensors;

import frc.robot.RobotCommander;
import frc.robot.RobotState;

public abstract class SensorBase {
    public abstract void updateState();

    public abstract void zeroSensor();

    public abstract void logData();
}