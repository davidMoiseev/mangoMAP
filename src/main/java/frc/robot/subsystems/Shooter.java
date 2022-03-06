package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.hotutilites.hotlogger.HotLogger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import frc.robot.RobotCommander;
import frc.robot.RobotState;
import static frc.robot.Constants.*;

public class Shooter extends SubsystemBase{
    RobotState robotState;
    TalonFX leftShooterMotor;
    TalonFX rightShooterMotor;
    private  DoubleSolenoid insidePneu;
    private  DoubleSolenoid outsidePneu;

    double pGain = 0.00026;
    double iGain = 0.00005;//Not good yet
    double dGain = 0.000008;
    double targetRPM = 0;
    double FF = 0;
    double targetVelocity_UnitsPer100ms;
    double RPM;
    int hoodPosition;

    public Shooter(RobotState robotState, PneumaticHub hub){
        this.robotState = robotState;
        leftShooterMotor = new TalonFX(LEFT_SHOOTER_MOTOR);
        rightShooterMotor = new TalonFX(RIGHT_SHOOTER_MOTOR);
        rightShooterMotor.configFactoryDefault();
        leftShooterMotor.configFactoryDefault();
        leftShooterMotor.setSelectedSensorPosition(0);
        rightShooterMotor.setSelectedSensorPosition(0);
        rightShooterMotor.follow(leftShooterMotor);
        rightShooterMotor.setInverted(true);
        leftShooterMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
        leftShooterMotor.configNeutralDeadband(0.001);
        leftShooterMotor.configNominalOutputForward(0, 30);
		leftShooterMotor.configNominalOutputReverse(0, 30);
		leftShooterMotor.configPeakOutputForward(1, 30);
		leftShooterMotor.configPeakOutputReverse(-1, 30);
        leftShooterMotor.config_kF(0, SHOOTER_FF, 30);
		leftShooterMotor.config_kP(0, SHOOTER_GAIN_P, 30);
		leftShooterMotor.config_kI(0, SHOOTER_GAIN_I, 30);
		leftShooterMotor.config_kD(0, SHOOTER_GAIN_D, 30);
        leftShooterMotor.setNeutralMode((NeutralMode.Coast));
        rightShooterMotor.setNeutralMode((NeutralMode.Coast));

        insidePneu = hub.makeDoubleSolenoid(SHOOTER_INSIDE_SOLENOID_FOWARD, SHOOTER_INSIDE_SOLENOID_REVERSE);
        outsidePneu = hub.makeDoubleSolenoid(SHOOTER_OUSIDE_SOLENOID_FOWARD, SHOOTER_OUTSIDE_SOLENOID_REVERSE);
    }

    @Override
    public void enabledAction(RobotState robotState, RobotCommander commander) {
        if (commander.getHoodPosition() == 1){
            inside(true);
            outside(true);
        }
        else if (commander.getHoodPosition() == 2){
            inside(true);
            outside(false);
        }
        else if (commander.getHoodPosition() == 3){
            inside(false);
            outside(true);
        }
        else if (commander.getHoodPosition() == 4){
            inside(true);
            outside(true);
        }
        else if (commander.getHoodPosition() == 5){
            inside(false);
            outside(false);
        }
        hoodPosition = commander.getHoodPosition();
        targetRPM = commander.getShooterSpeed();
        targetVelocity_UnitsPer100ms = (targetRPM * 2048) / 600;
        leftShooterMotor.set(TalonFXControlMode.Velocity, targetVelocity_UnitsPer100ms);
        RPM = (leftShooterMotor.getSelectedSensorVelocity() / 2048) * 600;
    }

    @Override
    public void disabledAction(RobotState robotState) {
        // this.robotState = robotState;
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
        HotLogger.Log("LeftShooterSpeed", (leftShooterMotor.getSelectedSensorVelocity() / 2048) * 600);
        SmartDashboard.putNumber("LeftShooterSpeed", (leftShooterMotor.getSelectedSensorVelocity() / 2048) * 600);
        HotLogger.Log("RightShooterSpeed", (rightShooterMotor.getSelectedSensorVelocity() / 2048) * 600);
        SmartDashboard.putNumber("RightShooterSpeed", (rightShooterMotor.getSelectedSensorVelocity() / 2048 ) * 600); 
        HotLogger.Log("TargetRPM", targetRPM);
        SmartDashboard.putNumber("TargetRPM", targetRPM); 
        HotLogger.Log("hoodPosition", hoodPosition);
        SmartDashboard.putNumber("hoodPosition", hoodPosition); 
    }

    public void inside(boolean tmp){
        Value x = Value.kOff;

        if(compBot){
            if (tmp){
                x = Value.kForward;
            }
            else {
                x = Value.kReverse;
            }
        } else {
            if (tmp){
                x = Value.kForward;
            }
            else {
                x = Value.kReverse;
            }
        }

        insidePneu.set(x);
    }
    public void outside(boolean tmp){
        Value x = Value.kOff;

        if(compBot){
            if (tmp){
                x = Value.kForward;
            }
            else {
                x = Value.kReverse;
            }
        } else {
            if (tmp){
                x = Value.kForward;
            }
            else {
                x = Value.kReverse;
            }
        }

        outsidePneu.set(x);
    }

    public boolean getShooterState() {
        if (targetRPM == 0) {
         return false;
        } 
        else if (Math.abs(targetRPM - RPM) < SHOOTER_OK_SPEED_TOLERANCE) {
            return true;
        } 
        else {
            return false;
        }
    }
    
}
