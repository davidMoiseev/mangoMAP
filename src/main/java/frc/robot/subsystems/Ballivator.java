package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.RobotCommander;
import frc.robot.RobotState;

import static frc.robot.Constants.*;

public class Ballivator extends SubsystemBase{
    RobotState robotState;
    CANSparkMax leftBallivatorMotor;
    CANSparkMax rightBallivatorMotor;

    public Ballivator(RobotState robotState){
        this.robotState = robotState;
        leftBallivatorMotor = new CANSparkMax(LEFT_BALLIVATOR_MOTOR, MotorType.kBrushless);
        rightBallivatorMotor = new CANSparkMax(RIGHT_BALLIVATOR_MOTOR, MotorType.kBrushless);
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
        // TODO Auto-generated method stub
        
    }
    
}
