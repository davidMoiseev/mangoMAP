package frc.robot.subsystems;

import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;
import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotCommander;
import frc.robot.RobotState;

import static frc.robot.Constants.*;

public class Ballivator extends SubsystemBase{
    RobotState robotState;
    CANSparkMax leftBallivatorMotor;
    CANSparkMax rightBallivatorMotor;
    DigitalInput ballSense;
    boolean ON;
    private Solenoid ballivatorSolenoid;
    Shooter shooter;
    Intake intake;

    public Ballivator(RobotState robotState, PneumaticHub hub, Shooter shooter, Intake intake){
        this.robotState = robotState;
        leftBallivatorMotor = new CANSparkMax(LEFT_BALLIVATOR_MOTOR, MotorType.kBrushless);
        rightBallivatorMotor = new CANSparkMax(RIGHT_BALLIVATOR_MOTOR, MotorType.kBrushless);
        ballSense = new DigitalInput(BALLIVATOR_SENSOR);
        ON = false;
        ballivatorSolenoid = hub.makeSolenoid(BALLIVATOR_SOLENOID);
        this.shooter = shooter;
        this.intake = intake;
        leftBallivatorMotor.setIdleMode(IdleMode.kBrake);
        rightBallivatorMotor.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void enabledAction(RobotState robotState, RobotCommander commander) {
        // TODO Auto-generated method stub
        // tmp {right T, left T, start, stick}
        // 
        boolean[] buttons = commander.getBallivator();
        
        if (buttons[4]){
            ON = false;
        }

        if (intake.getIntakeState()) {
            ON = true;
        } 
                
        if (ON) {
            if (ballSense.get()){
                setBallivatorSpeed(1, false, true);
            } else {
                setBallivatorSpeed(1, true, true);
            }
        }

        if (buttons[2] == true) {
            if (ballSense.get()){
                if (buttons[0]){
                    setBallivatorSpeed(1, true, false);
                }
                else if (buttons[1]){
                    setBallivatorSpeed(-1, true, true);
                }
                else {
                    setBallivatorSpeed(0, false, false);
                }
            }
            else {
                if (buttons[0]){
                    setBallivatorSpeed(1, true, true);
                }
                else if (buttons[1]){
                    setBallivatorSpeed(-1, true, true);
                }
                else{
                    setBallivatorSpeed(0, false, false);
                }
            }
        }

        else if (ON == false){
            setBallivatorSpeed(0, false, false);
        }

        if (robotState.isShooterReady()) {
        
            if (buttons[3]){
                setBallivatorSpeed(1, true, true);
                // if (COMP_BOT){
                //     ballivatorSolenoid.set(false);
                // } else {
                    ballivatorSolenoid.set(true);
                // }
                SmartDashboard.putBoolean("Gate Solenoid", ballivatorSolenoid.get());
            }
            else {
                if (ON) {
                    if (ballSense.get()){
                        setBallivatorSpeed(1, true, false);
                    } 
                    else {
                        setBallivatorSpeed(1, true, true);
                    }
                }
                else {
                    setBallivatorSpeed(0, false, false);
                }
                // if (COMP_BOT){
                //     ballivatorSolenoid.set(true);
                // } else {
                    ballivatorSolenoid.set(false);
                // }
            }
        } else{
            // if (COMP_BOT){
            //     ballivatorSolenoid.set(true);
            // } else {
                ballivatorSolenoid.set(false);
            // }        
        }

        SmartDashboard.putBoolean("Gate Solenoid", ballivatorSolenoid.get());
        SmartDashboard.putBoolean("Ballivator Sensei: ", ballSense.get());
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
    
    private void setBallivatorSpeed(double speed, boolean topOn, boolean btmOn){
        if (btmOn){
            rightBallivatorMotor.set(speed);
        }
        else {
            rightBallivatorMotor.set(0);
        }
        if (topOn){
            leftBallivatorMotor.set(speed);
        }
        else {
            leftBallivatorMotor.set(0);
        } 
    }
}
