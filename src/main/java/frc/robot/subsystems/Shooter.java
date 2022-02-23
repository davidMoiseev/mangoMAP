package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import org.hotutilites.hotlogger.HotLogger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PneumaticHub;
import frc.robot.RobotCommander;
import frc.robot.RobotState;
import static frc.robot.Constants.*;

public class Shooter extends SubsystemBase{
    RobotState robotState;
    TalonFX leftShooterMotor;
    TalonFX rightShooterMotor;

    public Shooter(RobotState robotState, PneumaticHub hub){
        this.robotState = robotState;
        leftShooterMotor = new TalonFX(LEFT_SHOOTER_MOTOR);
        rightShooterMotor = new TalonFX(RIGHT_SHOOTER_MOTOR);
    }

    @Override
    public void enabledAction(RobotState robotState, RobotCommander commander) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void disabledAction(RobotState robotState) {
        this.robotState = robotState;
        
    }

    @Override
    public void updateState() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void zeroActuators() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void zeroSensor() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void logData() {
        HotLogger.Log("LeftShooterSpeed", leftShooterMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber("LeftShooterSpeed", leftShooterMotor.getSelectedSensorVelocity());
        HotLogger.Log("RightShooterSpeed", rightShooterMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber("RightShooterSpeed", rightShooterMotor.getSelectedSensorVelocity()); 
    }
    
}
