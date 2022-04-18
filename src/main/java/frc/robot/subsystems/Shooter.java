package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.hotutilites.hotlogger.HotLogger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Timer;
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

    public enum Shot {
        NEUTRAL,
        FENDER,
        WALL,
        TARMACK,
        PROTECTED, AUTO, AUTO2, BLUEAUTO2, CHAOS
     , CLIMB, AUTO5B}

    double pGain = 0.00026;
    double iGain = 0.00005;//Not good yet
    double dGain = 0.000008;
    double targetRPM = 0;
    double FF = 0;
    double targetVelocity_UnitsPer100ms;
    double RPM;
    double shooterError = 0;
    private double shooterLatchTimer;
    private boolean shooterLatch;
    private int shooterSpeedTolerance = 9000;
    private double shooterTimeTolerance = 0;
    private Shot hoodPosition = Shot.NEUTRAL;

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
        boolean disableShooter;

        shooterSpeedTolerance = commander.getShooterSpeedThreshHold();
        shooterTimeTolerance = commander.getShooterTimeThreshHold();

        if (commander.getHoodPosition() == Shot.FENDER){
            inside(Piston.EXTEND);
            outside(Piston.RETRACT);
            targetRPM = SHOOTER_SPEED_FENDER;
            disableShooter = false;
            hoodPosition = Shot.FENDER;
        }
        else if (commander.getHoodPosition() == Shot.AUTO){
            inside(Piston.EXTEND);
            outside(Piston.RETRACT);
            targetRPM = SHOOTER_SPEED_AUTO;
            disableShooter = false;
            hoodPosition = Shot.AUTO;
        }
        else if(commander.getHoodPosition() == Shot.AUTO2){
            inside(Piston.RETRACT); // Extend
            outside(Piston.RETRACT); // Retract
            targetRPM = SHOOTER_SPEED_AUTO2;
            disableShooter = false;
            hoodPosition = Shot.AUTO2;
        }
        else if(commander.getHoodPosition() == Shot.BLUEAUTO2){
            inside(Piston.RETRACT); // Extend
            outside(Piston.RETRACT); // Retract
            targetRPM = SHOOTER_SPEED_BLUEAUTO2; // Or tarmack????
            disableShooter = false;
            hoodPosition = Shot.BLUEAUTO2;
        }
        else if (commander.getHoodPosition() == Shot.WALL){
            inside(Piston.RETRACT);
            outside(Piston.EXTEND);
            targetRPM = SHOOTER_SPEED_WALL;
            disableShooter = false;
            hoodPosition = Shot.WALL;
        }
        else if (commander.getHoodPosition() == Shot.TARMACK){
            inside(Piston.EXTEND);
            outside(Piston.RETRACT);
            targetRPM = SHOOTER_SPEED_TARMACK;
            disableShooter = false;
            hoodPosition = Shot.TARMACK;
        }
        else if (commander.getHoodPosition() == Shot.PROTECTED){
            inside(Piston.EXTEND);
            outside(Piston.EXTEND);
            targetRPM = SHOOTER_SPEED_PROTECTED;
            disableShooter = false;
            hoodPosition = Shot.PROTECTED;
        } else if (commander.getHoodPosition() == Shot.CLIMB){
            inside(Piston.RETRACT);
            outside(Piston.RETRACT);
            targetRPM = 0;
            disableShooter = true;
            hoodPosition = Shot.CLIMB;
        }else if(commander.getHoodPosition() == Shot.AUTO5B){
            inside(Piston.RETRACT);
            outside(Piston.EXTEND);
            targetRPM = SHOOTER_SPEED_AUTO5B;
            disableShooter = false;
            hoodPosition = Shot.AUTO5B;
        } else if (commander.getHoodPosition() == Shot.CHAOS){
            inside(Piston.EXTEND);
            outside(Piston.EXTEND);
            targetRPM = SHOOTER_SPEED_FENDER;
            disableShooter = true;
            hoodPosition = Shot.CHAOS;
        } else {
            inside(Piston.OFF);
            outside(Piston.OFF);
            targetRPM = 0;
            disableShooter = true;
        }
        robotState.setShooterOn(!disableShooter);
        if(targetRPM == 0){
            leftShooterMotor.set(TalonFXControlMode.PercentOutput, 0);
        } else {
            targetVelocity_UnitsPer100ms = (targetRPM * 2048) / 600;
            if ( ! commander.getOverrideShooterMotor() || disableShooter) {
            leftShooterMotor.set(TalonFXControlMode.Velocity, targetVelocity_UnitsPer100ms);
            } else  {
            leftShooterMotor.set(TalonFXControlMode.PercentOutput, 0.0);
            }
        }
    }

    @Override
    public void disabledAction(RobotState robotState) {
        // this.robotState = robotState;
    }

    @Override
    public void updateState() {
        RPM = (leftShooterMotor.getSelectedSensorVelocity() / 2048) * 600;
            if (targetRPM == 0) {
                robotState.setShooterReady(false);
                shooterLatch = false;
                shooterLatchTimer = 0;
            }  else if (shooterLatch) {
                robotState.setShooterReady(true);
            }  else if (Math.abs(targetRPM - RPM) < shooterSpeedTolerance) {
                shooterLatchTimer = shooterLatchTimer + .02;
                if (shooterLatchTimer >= shooterTimeTolerance) {
                    shooterLatch = true;
                }    
            } else {
                robotState.setShooterReady(false);
                shooterLatch = false;
            }

            robotState.setActualShooterSpeed(RPM);
            robotState.setTargetShooterSpeed(targetRPM);
            SmartDashboard.putNumber("targetRPM", targetRPM);
            SmartDashboard.putNumber("RPM", RPM);
            SmartDashboard.putBoolean("shooterReady", robotState.isShooterReady());
            SmartDashboard.putNumber("shooterLatchTimer",shooterLatchTimer);
            shooterError = leftShooterMotor.getClosedLoopError();

    }

    @Override
    public void zeroActuators() {
        // TODO Auto-generated method stub
        targetRPM = 0;
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
        HotLogger.Log("hoodPosition", ""+hoodPosition);
        SmartDashboard.putString("hoodPosition", ""+hoodPosition); 
        HotLogger.Log("shooterError", shooterError);
    }

    enum Piston {
        OFF,
        RETRACT,
        EXTEND
    }

    public void inside(Piston tmp){
        Value x = Value.kOff;

        if(COMP_BOT){
            if (tmp == Piston.EXTEND){
                x = Value.kForward;
            }
            else if (tmp == Piston.RETRACT) {
                x = Value.kReverse;
            } else {
                x = Value.kOff;
            }
        } else {
            if (tmp == Piston.EXTEND){
                x = Value.kReverse;
            }
            else if (tmp == Piston.RETRACT) {
                x = Value.kForward;
            } else {
                x = Value.kOff;
            }
        }

        insidePneu.set(x);
    }
    public void outside(Piston tmp){
        Value x = Value.kOff;

        if(COMP_BOT){
            if (tmp == Piston.EXTEND){
                x = Value.kForward;
            }
            else if (tmp == Piston.RETRACT) {
                x = Value.kReverse;
            } else {
                x = Value.kOff;
            }
        } else {
            if (tmp == Piston.EXTEND){
                x = Value.kReverse;
            }
            else if (tmp == Piston.RETRACT) {
                x = Value.kForward;
            } else {
                x = Value.kOff;
            }
        }

        outsidePneu.set(x);
    } 
}
