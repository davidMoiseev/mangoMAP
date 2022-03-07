package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import org.hotutilites.hotlogger.HotLogger;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import edu.wpi.first.wpilibj.PneumaticHub;
import frc.robot.RobotCommander;
import frc.robot.RobotState;
import static frc.robot.Constants.*;

public class Climber extends SubsystemBase{
    private TalonFX climberMotor;
    private DoubleSolenoid climberExtend;
    private Solenoid climberRelease;
    RobotState robotState;
    public double targetPosTicks = (PACKAGE_ANGLE * CLIMBER_TICKS_PER_DEGREE);
    public double targetPosDeg = (PACKAGE_ANGLE);
    public double actualPosTicks;
    public double actualPosDeg;
    public boolean manualControlFlag = false;
    long latchTimer;
    boolean unlatchTrigger = false;
    public int climberState = 1;
      /*    1:  Climber Retracted
            2:  Climber Extended
            3:  Moving Arm to vertical
            4:  Progress to 3rd Bar
            5:  Unload/Release 2nd Bar
            6:  Progress to 4th Bar
            7:  Unload/Release 3rd Bar
            8:  Move to final State
            */

    public Climber(RobotState robotState, PneumaticHub hub) {
        this.robotState = robotState;
        climberMotor = new TalonFX(CLIMBER_MOTOR);
        climberExtend = hub.makeDoubleSolenoid(CLIMBER_EXTEND_FWD_SOLENOID, CLIMBER_EXTEND_REV_SOLENOID);
        climberRelease = hub.makeSolenoid(CLIMBER_RELEASE_SOLENOID);
        climberMotor.configFactoryDefault();
        climberMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
        climberMotor.configNominalOutputForward(0, 30);
		climberMotor.configNominalOutputReverse(0, 30);
		climberMotor.configPeakOutputForward(1, 30);
		climberMotor.configPeakOutputReverse(-1, 30); 
        climberMotor.selectProfileSlot(0, 0);
		climberMotor.config_kF(0, CLIMBER_F, 30);
		climberMotor.config_kP(0, CLIMBER_P, 30);
		climberMotor.config_kI(0, CLIMBER_I, 30);
		climberMotor.config_kD(0, CLIMBER_D, 30);
        climberMotor.configNeutralDeadband(0.001, 30);
        climberMotor.setSensorPhase(false);
		climberMotor.setInverted(false);
        climberMotor.configMotionCruiseVelocity(CLIMBER_CRUISE_VELOCITY, 30);
        climberMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 30);
		climberMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 30);
		climberMotor.configMotionAcceleration(CLIMBER_CRUISE_ACCELERATION, 30);
        climberMotor.setSelectedSensorPosition(degreeToTicks(PACKAGE_ANGLE), 0, 30);
        climberMotor.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void enabledAction(RobotState robotState, RobotCommander commander) {

        if (COMP_BOT) {
            if (commander.getClimberExtend()) {
                robotState.setClimberExtended(true);
                climberExtend.set(DoubleSolenoid.Value.kForward);
            }
            
            if (commander.getClimberRetract()) {
                robotState.setClimberExtended(false);
                climberExtend.set(DoubleSolenoid.Value.kReverse);
            }
        } else {
            if (commander.getClimberExtend()) {
                robotState.setClimberExtended(true);
                climberExtend.set(DoubleSolenoid.Value.kReverse);
            }

            if (commander.getClimberRetract()) {
                robotState.setClimberExtended(false);
                climberExtend.set(DoubleSolenoid.Value.kForward);
            }
        }

        if (commander.getClimberExtend()) {
            robotState.setClimberExtended(true);
            climberExtend.set(DoubleSolenoid.Value.kReverse);
        }
        
        if (commander.getClimberRetract()) {
            robotState.setClimberExtended(false);
            climberExtend.set(DoubleSolenoid.Value.kForward);
        }

        if (commander.getClimberManualControl() == true) {
            climberMotor.set(TalonFXControlMode.PercentOutput, commander.getClimberMotor());
            climberRelease.set(commander.getClimberRelease());
            manualControlFlag = true;
        } else {
            manualControlFlag = false;
            if (climberState == 1) {  // Climber Retracted
                if (robotState.getClimberExtended() == true) {
                    climberState = 2;
                    latchTimer = System.currentTimeMillis();
                    unlatchTrigger = true;
                    targetPosDeg = PACKAGE_ANGLE;
                }
            } else if (climberState == 2) { // Climber Extended
                if (robotState.getClimberExtended() == false) {
                    climberState = 1;
                    unlatchTrigger = false;
                } else if ((System.currentTimeMillis() - latchTimer) > CLIMBER_EXTEND_TIME){
                    unlatchTrigger = false;
                    climberState = 3;
                }
                climberRelease.set(commander.getClimberRelease());
                targetPosDeg = PACKAGE_ANGLE;

            } else if (climberState == 3) { // Moving Arm to vertical
                if ((commander.getBbuttonHeld() == false) && 
                    (ticksToDegrees(climberMotor.getSelectedSensorPosition()) > (CLIMBER_STATE3_ANGLE - CLIMBER_ANGLE_HYSTERESIS))) {
                        climberState = 4;
                }
                if ((commander.getBbuttonHeld() == true)) {
                    climberMotor.set(TalonFXControlMode.MotionMagic, degreeToTicks(CLIMBER_STATE3_ANGLE));
                    targetPosDeg = CLIMBER_STATE3_ANGLE;
                }   

            } else if (climberState == 4) { // Progress to 3rd Bar
                if ((commander.getAbuttonHeld() == false) && 
                    (ticksToDegrees(climberMotor.getSelectedSensorPosition()) > (CLIMBER_STATE4_ANGLE - CLIMBER_ANGLE_HYSTERESIS))) {
                        climberState = 5;
                }
                if ((commander.getAbuttonHeld() == true)) {
                    climberMotor.set(TalonFXControlMode.MotionMagic, degreeToTicks(CLIMBER_STATE4_ANGLE));
                    targetPosDeg = CLIMBER_STATE4_ANGLE;
                }

            } else if (climberState == 5) { // Unload/Release 2nd Bar
                if ((commander.getBbuttonHeld() == false) &&
                    (unlatchTrigger == false) &&
                    (ticksToDegrees(climberMotor.getSelectedSensorPosition()) < (CLIMBER_STATE5_ANGLE + CLIMBER_ANGLE_HYSTERESIS))) {
                        latchTimer = System.currentTimeMillis();
                        climberRelease.set(true);
                        unlatchTrigger = true;
                }
                if ((unlatchTrigger == true) && ((System.currentTimeMillis() - latchTimer) > CLIMBER_LATCH_RELEASE_TIME)) {
                    climberState = 6;
                    unlatchTrigger = false;
                    climberRelease.set(false);
                }
                if ((commander.getBbuttonHeld() == true)) {
                    climberMotor.set(TalonFXControlMode.MotionMagic, degreeToTicks(CLIMBER_STATE5_ANGLE));
                    targetPosDeg = CLIMBER_STATE5_ANGLE;
                }

            } else if (climberState == 6) { // Progress to 4th Bar
                if ((commander.getAbuttonHeld() == false) && 
                    (ticksToDegrees(climberMotor.getSelectedSensorPosition()) > (CLIMBER_STATE6_ANGLE - CLIMBER_ANGLE_HYSTERESIS))) {
                        climberState = 7;
                }
                if ((commander.getAbuttonHeld() == true)) {
                    climberMotor.set(TalonFXControlMode.MotionMagic, degreeToTicks(CLIMBER_STATE6_ANGLE));
                    targetPosDeg = CLIMBER_STATE6_ANGLE;
                }
                
            } else if (climberState == 7) { // Unload/Release 3rd Bar
                if ((commander.getBbuttonHeld() == false) && 
                    (unlatchTrigger == false) &&
                    (ticksToDegrees(climberMotor.getSelectedSensorPosition()) < (CLIMBER_STATE7_ANGLE + CLIMBER_ANGLE_HYSTERESIS))) {
                        latchTimer = System.currentTimeMillis();
                        climberRelease.set(true);
                        unlatchTrigger = true;
                }
                if ((unlatchTrigger == true) && ((System.currentTimeMillis() - latchTimer) > CLIMBER_LATCH_RELEASE_TIME)) {
                    climberState = 8;
                    unlatchTrigger = false;
                    climberRelease.set(false);
                }
                if ((commander.getBbuttonHeld() == true)) {
                    climberMotor.set(TalonFXControlMode.MotionMagic, degreeToTicks(CLIMBER_STATE7_ANGLE));
                    targetPosDeg = CLIMBER_STATE7_ANGLE;
                }
            } else if (climberState == 8) { // Move to final State
                climberMotor.set(TalonFXControlMode.MotionMagic, degreeToTicks(CLIMBER_STATE8_ANGLE));
                targetPosDeg = CLIMBER_STATE8_ANGLE;
            }

        }
        targetPosTicks = degreeToTicks(targetPosDeg);
        actualPosTicks = climberMotor.getSelectedSensorPosition();
        actualPosDeg = ticksToDegrees(actualPosTicks);

    }

    @Override
    public void disabledAction(RobotState robotState) {
        climberMotor.set(TalonFXControlMode.PercentOutput, 0.0);
    }

    @Override
    public void updateState() {

    }

    @Override
    public void zeroActuators() {

    }

    @Override
    public void zeroSensor() {

    }

    @Override
    public void logData() {
        HotLogger.Log("ClimberCmd", climberMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("ClimberCmd", climberMotor.getMotorOutputPercent());
        HotLogger.Log("ClimberExtended", robotState.getClimberExtended());
        SmartDashboard.putBoolean("ClimberExtended", robotState.getClimberExtended());
        SmartDashboard.putNumber("targetPosDeg", targetPosDeg);
        SmartDashboard.putNumber("targetPosTicks", targetPosTicks);
        SmartDashboard.putNumber("ClimberSpeed", (climberMotor.getSelectedSensorVelocity()/100.0));
        SmartDashboard.putNumber("actualPosTicks", climberMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("actualPosDeg", actualPosDeg);
        SmartDashboard.putNumber("climberState", climberState);
        HotLogger.Log("climberState", climberState);
        HotLogger.Log("targetPosDeg", targetPosDeg);
        HotLogger.Log("targetPosTicks", targetPosTicks);
        HotLogger.Log("actualPosTicks", climberMotor.getSelectedSensorPosition());
        HotLogger.Log("actualPosDeg", actualPosDeg);
        SmartDashboard.putBoolean("manualControlFlag", manualControlFlag);

    }

    private double degreeToTicks(double degrees){
        return ((347.222 * degrees) / 360) * 2048;
    }
    private double ticksToDegrees(double ticks){
        return ((ticks / 2048) * 360) / 347.222;
    }
}
