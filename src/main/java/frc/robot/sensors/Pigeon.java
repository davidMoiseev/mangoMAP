package frc.robot.sensors;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;

import org.hotutilites.hotlogger.HotLogger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.AutonCommader;
import frc.robot.Robot;
import frc.robot.RobotCommander;
import frc.robot.RobotState;

import static frc.robot.Constants.*;

public class Pigeon extends SensorBase{

    private RobotState robotState;

    private double theta;

    private Pigeon2 pigeon;

    public Pigeon(RobotState robotState) {
        this.robotState = robotState;

        pigeon = new Pigeon2(DRIVETRAIN_PIGEON_ID);
    }

    @Override
    public void updateState() {
        theta = pigeon.getYaw();
        robotState.setRotation2d(theta);
    }

    @Override
    public void zeroSensor() {
        pigeon.setYaw(0);
    }

    @Override
    public void logData() {
        // TODO Auto-generated method stub
        SmartDashboard.putNumber("Theta", theta);

        HotLogger.Log("Theta", theta);
    }

    public void initializeAuton(AutonCommader selectedAuton) {
        pigeon.setYaw(selectedAuton.getInitialPose().getRotation().getDegrees());

        SmartDashboard.putNumber("InitializedPigeon", selectedAuton.getInitialPose().getRotation().getDegrees());
    }

    public void enabledAction(RobotCommander commander) {
        if (commander.getResetIMU()) {
            pigeon.setYaw(commander.getResetIMUAngle());
        }
    }
    
}
