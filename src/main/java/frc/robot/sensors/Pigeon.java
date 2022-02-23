package frc.robot.sensors;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;

import org.hotutilites.hotlogger.HotLogger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    public void updateState(RobotState robotState, RobotCommander commander) {
        if(commander.getResetIMU()){
            zeroSensor();
        }

        theta = pigeon.getYaw();
        robotState.setTheta(theta);
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
    
}
