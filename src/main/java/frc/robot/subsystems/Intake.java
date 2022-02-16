package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.RobotCommander;
import frc.robot.RobotState;

import static frc.robot.Constants.*;

public class Intake extends SubsystemBase {

    RobotState robotState;

    TalonFX rightIntake;
    TalonFX leftIntake;

    public Intake(RobotState robotState){
        this.robotState = robotState;

        rightIntake = new TalonFX(RIGHT_INTAKE_MOTOR);
        leftIntake = new TalonFX(LEFT_INTAKE_MOTOR);
    }

    @Override
    public void enabledAction(RobotState robotState, RobotCommander commander) {
        if(commander.getRunLeftIntake()){
            leftIntake.set(ControlMode.PercentOutput, .75);
        } else {
            leftIntake.set(ControlMode.PercentOutput, 0);
        }
        if(commander.getRunRightIntake()){
            rightIntake.set(ControlMode.PercentOutput, .75);
        } else {
            rightIntake.set(ControlMode.PercentOutput, 0);
        }
    }

    @Override
    public void disabledAction(RobotState robotState) {
        // TODO Auto-generated method stub
        
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
