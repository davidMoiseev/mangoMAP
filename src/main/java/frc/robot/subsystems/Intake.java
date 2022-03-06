package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.PneumaticHub;
import frc.robot.RobotCommander;
import frc.robot.RobotState;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import org.hotutilites.hotlogger.HotLogger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.Constants.*;

public class Intake extends SubsystemBase {
    RobotState robotState;
    TalonFX rightIntakeMotor;
    TalonFX leftIntakeMotor;
    DoubleSolenoid rightIntakeSolenoid;
    DoubleSolenoid leftIntakeSolenoid;

    public Intake(RobotState robotState, PneumaticHub hub){
        this.robotState = robotState;

        rightIntakeMotor = new TalonFX(RIGHT_INTAKE_MOTOR);
        leftIntakeMotor = new TalonFX(LEFT_INTAKE_MOTOR);
        rightIntakeSolenoid = hub.makeDoubleSolenoid(RIGHT_INTAKE_FWD_SOLENOID, RIGHT_INTAKE_REV_SOLENOID);
        leftIntakeSolenoid = hub.makeDoubleSolenoid(LEFT_INTAKE_FWD_SOLENOID, LEFT_INTAKE_REV_SOLENOID);
    }
    
    boolean intakeState = false;

    @Override
    public void enabledAction(RobotState robotState, RobotCommander commander) {
        if(compBot){
            if (commander.getRunLeftIntake()) {
                if (commander.getRunLeftIntake()) {
                    leftIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
                    leftIntakeMotor.set(ControlMode.PercentOutput, .85);
                } else {
                    leftIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
                    leftIntakeMotor.set(ControlMode.PercentOutput, 0.0);
                } 
            }

            if (commander.getRunRightIntake()) {
                rightIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
                rightIntakeMotor.set(ControlMode.PercentOutput, -.85);
            } else {
                rightIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
                rightIntakeMotor.set(ControlMode.PercentOutput, 0.0);
            }
            if (commander.getRunLeftIntake() || commander.getRunRightIntake()) {
                intakeState = true;
            } else {
                intakeState = false;
            }
        } else {
            if (commander.getRunLeftIntake()) {
                leftIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
                leftIntakeMotor.set(ControlMode.PercentOutput, .85);
            } else {
                leftIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
                leftIntakeMotor.set(ControlMode.PercentOutput, 0.0);
            }

            if (commander.getRunRightIntake()) {
                rightIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
                rightIntakeMotor.set(ControlMode.PercentOutput, -.85);
            } else {
                rightIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
                rightIntakeMotor.set(ControlMode.PercentOutput, 0.0);
            }
            if (commander.getRunLeftIntake() || commander.getRunRightIntake()) {
                intakeState = true;
            } else {
                intakeState = false;
            }
        }


    }

    @Override
    public void disabledAction(RobotState robotState) {
        rightIntakeMotor.set(ControlMode.PercentOutput, 0.0);
        leftIntakeMotor.set(ControlMode.PercentOutput, 0.0);
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
        HotLogger.Log("LeftIntakeCmd", leftIntakeMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("LeftIntakeCmd", leftIntakeMotor.getMotorOutputPercent());
        HotLogger.Log("RightIntakeCmd", rightIntakeMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("RightIntakeCmd", rightIntakeMotor.getMotorOutputPercent()); 
    }
    public boolean getIntakeState(){
        return intakeState;
    }
}
