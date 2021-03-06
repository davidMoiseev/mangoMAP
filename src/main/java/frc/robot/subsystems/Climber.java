package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
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
    private double climberSpeed;
    long latchTimer;
    boolean unlatchTrigger = false;

    double rampVal = 0;

    double timer = 0;

    //public int climberState = 1;
      /*    1:  Climber Retracted
            2:  Climber Extended
            3:  Moving Arm to vertical
            4:  Progress to 3rd Bar
            5:  Unload/Release 2nd Bar
            6:  Progress to 4th Bar
            7:  Unload/Release 3rd Bar
            8:  Move to final State
            9:  Process End
            */

    enum climbState {
        preClimb,
        latchPosition,
        climbToMid,
        rampToApproachSpeed,
        ApproachSpeedWithVelocityDetection,
        prepareToRelease,
        release,
        zero,
        climbToTraversal,        
        rampToApproachSpeed2,
        ApproachSpeedWithVelocityDetection2,
        prepareToReleaseFromTraversal,
        releaseFromTraversal,
        finalMove,
        waitForRelease1,
        waitForRelease2
    }
    
    public climbState climberState=climbState.preClimb;

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
        climberMotor.setSensorPhase((COMP_BOT) ? false : true);
		climberMotor.setInverted((COMP_BOT) ? false : true);
        climberMotor.configMotionCruiseVelocity(CLIMBER_CRUISE_VELOCITY, 30);
        climberMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 30);
		climberMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 30);
		climberMotor.configMotionAcceleration(CLIMBER_CRUISE_ACCELERATION, 30);
        climberMotor.setSelectedSensorPosition(degreeToTicks(PACKAGE_ANGLE), 0, 30);
        climberMotor.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void enabledAction(RobotState robotState, RobotCommander commander) {
        climberMotor.setNeutralMode(NeutralMode.Brake);

        // if (COMP_BOT) {
        //     if (commander.getClimberExtend()) {
        //         robotState.setClimberExtended(true);
        //         climberExtend.set(DoubleSolenoid.Value.kForward);
        //     }
            
        //     if (commander.getClimberRetract()) {
        //         robotState.setClimberExtended(false);
        //         climberExtend.set(DoubleSolenoid.Value.kReverse);
        //     }
        // } else {
        //     if (commander.getClimberExtend()) {
        //         robotState.setClimberExtended(true);
        //         climberExtend.set(DoubleSolenoid.Value.kReverse);
        //     }

        //     if (commander.getClimberRetract()) {
        //         robotState.setClimberExtended(false);
        //         climberExtend.set(DoubleSolenoid.Value.kForward);
        //     }
        // }

        if (commander.getClimberExtend()) {
            robotState.setClimberExtended(true);
        }
        
        if (commander.getClimberRetract()) {
            robotState.setClimberExtended(false);
            climberExtend.set(DoubleSolenoid.Value.kReverse);
        }

        if(robotState.getClimberExtended()){
            climberMotor.set(ControlMode.PercentOutput, .5);

            if(actualPosDeg > -60){
                climberMotor.set(ControlMode.PercentOutput, 0);
                climberExtend.set(DoubleSolenoid.Value.kForward);
            }
        }

        if (commander.getClimberManualControl() == true) {
            climberMotor.set(TalonFXControlMode.PercentOutput, commander.getClimberMotor());
            climberRelease.set(commander.getClimberRelease());
            manualControlFlag = true;
        } else if (robotState.getClimberExtended() && manualControlFlag == false) {
        //     manualControlFlag = false;
        //     if (climberState == 1) {  // Climber Retracted
        //         if (robotState.getClimberExtended() == true) {
        //             climberState = 2;
        //             latchTimer = System.currentTimeMillis();
        //             unlatchTrigger = true;
        //             targetPosDeg = PACKAGE_ANGLE;
        //         }
        //     } else if (climberState == 2) { // Climber Extended
        //         if (robotState.getClimberExtended() == false) {
        //             climberState = 1;
        //             unlatchTrigger = false;
        //         } else if ((System.currentTimeMillis() - latchTimer) > CLIMBER_EXTEND_TIME){
        //             unlatchTrigger = false;
        //             climberState = 3;
        //         }
        //         climberRelease.set(commander.getClimberRelease());
        //         targetPosDeg = PACKAGE_ANGLE;

        //     } else if (climberState == 3) { // Moving Arm to vertical
        //         if ((commander.getBbuttonHeld() == false) && 
        //             (ticksToDegrees(climberMotor.getSelectedSensorPosition()) > (CLIMBER_STATE3_ANGLE - CLIMBER_ANGLE_HYSTERESIS))) {
        //                 climberState = 4;
        //         }
        //         if ((commander.getBbuttonHeld() == true)) {
        //             climberMotor.set(TalonFXControlMode.MotionMagic, degreeToTicks(CLIMBER_STATE3_ANGLE));
        //             targetPosDeg = CLIMBER_STATE3_ANGLE;
        //         }   

        //     } else if (climberState == 4) { // Progress to 3rd Bar
        //         if ((commander.getAbuttonHeld() == false) && 
        //             (ticksToDegrees(climberMotor.getSelectedSensorPosition()) > (CLIMBER_STATE4_ANGLE - CLIMBER_ANGLE_HYSTERESIS))) {
        //                 climberState = 5;
        //         }
        //         if ((commander.getAbuttonHeld() == true)) {
        //             climberMotor.set(TalonFXControlMode.MotionMagic, degreeToTicks(CLIMBER_STATE4_ANGLE));
        //             targetPosDeg = CLIMBER_STATE4_ANGLE;
        //         }

        //     } else if (climberState == 5) { // Unload/Release 2nd Bar
        //         if ((commander.getBbuttonHeld() == false) &&
        //             (unlatchTrigger == false) &&
        //             (ticksToDegrees(climberMotor.getSelectedSensorPosition()) < (CLIMBER_STATE5_ANGLE + CLIMBER_ANGLE_HYSTERESIS))) {
        //                 latchTimer = System.currentTimeMillis();
        //                 climberRelease.set(true);
        //                 unlatchTrigger = true;
        //         }
        //         if ((unlatchTrigger == true) && ((System.currentTimeMillis() - latchTimer) > CLIMBER_LATCH_RELEASE_TIME)) {
        //             climberState = 6;
        //             unlatchTrigger = false;
        //             climberRelease.set(false);
        //         }
        //         if ((commander.getBbuttonHeld() == true)) {
        //             climberMotor.set(TalonFXControlMode.MotionMagic, degreeToTicks(CLIMBER_STATE5_ANGLE));
        //             targetPosDeg = CLIMBER_STATE5_ANGLE;
        //         }

        //     } else if (climberState == 6) { // Progress to 4th Bar
        //         if ((commander.getAbuttonHeld() == false) && 
        //             (ticksToDegrees(climberMotor.getSelectedSensorPosition()) > (CLIMBER_STATE6_ANGLE - CLIMBER_ANGLE_HYSTERESIS))) {
        //                 climberState = 7;
        //         }
        //         if ((commander.getAbuttonHeld() == true)) {
        //             climberMotor.set(TalonFXControlMode.MotionMagic, degreeToTicks(CLIMBER_STATE6_ANGLE));
        //             targetPosDeg = CLIMBER_STATE6_ANGLE;
        //         }
                
        //     } else if (climberState == 7) { // Unload/Release 3rd Bar
        //         if ((commander.getBbuttonHeld() == false) && 
        //             (unlatchTrigger == false) &&
        //             (ticksToDegrees(climberMotor.getSelectedSensorPosition()) < (CLIMBER_STATE7_ANGLE + CLIMBER_ANGLE_HYSTERESIS))) {
        //                 latchTimer = System.currentTimeMillis();
        //                 climberRelease.set(true);
        //                 unlatchTrigger = true;
        //         }
        //         if ((unlatchTrigger == true) && ((System.currentTimeMillis() - latchTimer) > CLIMBER_LATCH_RELEASE_TIME)) {
        //             climberState = 8;
        //             unlatchTrigger = false;
        //             latchTimer = System.currentTimeMillis();
        //             climberRelease.set(false);
        //         }
        //         if ((commander.getBbuttonHeld() == true)) {
        //             climberMotor.set(TalonFXControlMode.MotionMagic, degreeToTicks(CLIMBER_STATE7_ANGLE));
        //             targetPosDeg = CLIMBER_STATE7_ANGLE;
        //         }
        //     } else if (climberState == 8) { // Move to final State
        //         climberMotor.set(TalonFXControlMode.MotionMagic, degreeToTicks(CLIMBER_STATE8_ANGLE));
        //         targetPosDeg = CLIMBER_STATE8_ANGLE;
        //         if ((System.currentTimeMillis() - latchTimer) > CLIMBER_LATCH_END_TIME) {
        //             climberState = 9;
        //         }

        //     } else if (climberState == 9) { // Move to End State
        //         climberMotor.set(TalonFXControlMode.PercentOutput, 0.0);
        //     }
            if (climberState==climbState.preClimb) {
                if(commander.getAbuttonHeld() || commander.getYButtonHeld()){
                    climberState = climbState.latchPosition;
                }
            }
            if (climberState==climbState.latchPosition) {
                climberMotor.set(TalonFXControlMode.MotionMagic, degreeToTicks(CLIMBER_STATE3_ANGLE));
                targetPosDeg = CLIMBER_STATE3_ANGLE;

                if(actualPosDeg >= CLIMBER_STATE3_ANGLE - .5){
                    if(commander.getBbuttonHeld() || commander.getYButtonHeld()){
                        climberState = climbState.climbToMid;
                    }
                }
            }

            if (climberState==climbState.climbToMid) {
                climberMotor.set(ControlMode.PercentOutput, 1);
                if(actualPosDeg > 110){
                    climberState = climbState.rampToApproachSpeed;
                    rampVal = 0.75;
                }
            }

            if (climberState==climbState.rampToApproachSpeed) {
                climberMotor.set(ControlMode.PercentOutput, rampVal);
                if(rampVal > .6){
                    rampVal -= .05;
                }
                if(actualPosDeg > 130){
                    climberState = climbState.ApproachSpeedWithVelocityDetection;
                    timer = 0;
                }
            }

            if (climberState==climbState.ApproachSpeedWithVelocityDetection) {
                climberMotor.set(ControlMode.PercentOutput, .6);
                if(actualPosDeg < 153){
                    if(Math.abs(robotState.getPitchSpeed() - climberSpeed) < 10){
                        timer++;
                        if(timer >= 10){
                            climberState = climbState.prepareToRelease;
                        }
                    } else {
                        timer = 0;
                    }
                } else {
                    climberState = climbState.prepareToRelease;
                }
            }

            // if(climberState == climbState.waitForFirstRelease){
            //     climberMotor.set(ControlMode.PercentOutput, 0);
            //     if(commander.getAbuttonHeld()){
            //         climberState = climbState.prepareToRelease;
            //     }
            // }

            if (climberState==climbState.prepareToRelease) {
                // climberMotor.set(TalonFXControlMode.MotionMagic, degreeToTicks(CLIMBER_STATE5_ANGLE));
                // targetPosDeg = CLIMBER_STATE5_ANGLE;
                // if(commander.getAbuttonHeld()){
                //     climberState = climbState.release;
                // }

                if(actualPosDeg > CLIMBER_STATE5_ANGLE){
                    climberMotor.set(ControlMode.PercentOutput, -.5);
                } else {
                    climberMotor.set(ControlMode.PercentOutput, 0);
                    if(commander.getAbuttonHeld() || commander.getYButtonHeld()){
                        climberState = climbState.release;
                    }
                }
            }

            if (climberState==climbState.release) {
                climberRelease.set(true);
                climberState = climbState.waitForRelease1;
                timer = 0;
            }

            if(climberState == climbState.waitForRelease1){
                if(timer > 25){
                    climberRelease.set(false);

                    if (commander.getBbuttonHeld() || commander.getYButtonHeld()){
                        climberState = climbState.climbToTraversal;
                    }
                }
                timer++;
            }

            if (climberState==climbState.climbToTraversal) {
                climberMotor.set(ControlMode.PercentOutput, 1);

                if(actualPosDeg > 290){
                    climberState = climbState.rampToApproachSpeed2;
                    rampVal = 0.75;
                }
            }

            if (climberState==climbState.rampToApproachSpeed2) {
                climberMotor.set(ControlMode.PercentOutput, rampVal);
                if(rampVal > .6){
                    rampVal -= .05;
                }
                if(actualPosDeg > 310){
                    climberState = climbState.ApproachSpeedWithVelocityDetection2;
                    timer = 0;
                }
            }

            if (climberState==climbState.ApproachSpeedWithVelocityDetection2) {
                climberMotor.set(ControlMode.PercentOutput, .6);
                if(actualPosDeg < 332){
                    if(Math.abs(robotState.getPitchSpeed() - climberSpeed) < 10){
                        timer++;
                        if(timer >= 10){
                            climberState = climbState.prepareToReleaseFromTraversal;
                        }
                    } else {
                        timer = 0;
                    }
                } else {
                    climberState = climbState.prepareToReleaseFromTraversal;
                }
            }

            if (climberState==climbState.prepareToReleaseFromTraversal) {
                // climberMotor.set(TalonFXControlMode.MotionMagic, degreeToTicks(CLIMBER_STATE7_ANGLE));
                // targetPosDeg = CLIMBER_STATE7_ANGLE;
                // if(commander.getAbuttonHeld()){
                //     climberState = climbState.releaseFromTraversal;
                // }

                if(actualPosDeg > CLIMBER_STATE7_ANGLE){
                    climberMotor.set(ControlMode.PercentOutput, -.5);
                } else {
                    climberMotor.set(ControlMode.PercentOutput, 0);
                    if(commander.getAbuttonHeld() || commander.getYButtonHeld()){
                        climberState = climbState.releaseFromTraversal;
                    }
                }
            }

            if (climberState==climbState.releaseFromTraversal) {
                climberRelease.set(true);
                climberState = climbState.waitForRelease2;
                timer = 0;
            }

            if(climberState == climbState.waitForRelease2){
                if(timer > 25){
                    climberRelease.set(false);

                    if (commander.getBbuttonHeld() || commander.getYButtonHeld()){
                        climberState = climbState.finalMove;
                    }
                }
                timer++;
            }

            if (climberState==climbState.finalMove) {
                climberMotor.set(TalonFXControlMode.MotionMagic, degreeToTicks(CLIMBER_STATE8_ANGLE));
                targetPosDeg = CLIMBER_STATE8_ANGLE;
                if (ticksToDegrees(climberMotor.getSelectedSensorPosition()) > CLIMBER_STATE8_ANGLE - 5){
                    climberState = climbState.zero;
                }
            }

            if(climberState == climbState.zero){
                climberMotor.set(ControlMode.PercentOutput, 0);
            }

        } else if (manualControlFlag){
            climberMotor.set(ControlMode.PercentOutput, 0);
        }
        targetPosTicks = degreeToTicks(targetPosDeg);
        actualPosTicks = climberMotor.getSelectedSensorPosition();
        climberSpeed = ticksToDegrees(climberMotor.getSelectedSensorVelocity() * 10);
        actualPosDeg = ticksToDegrees(actualPosTicks);
    }

    @Override
    public void disabledAction(RobotState robotState) {
        climberMotor.set(TalonFXControlMode.PercentOutput, 0.0);
        climberMotor.setNeutralMode(NeutralMode.Coast);
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
        HotLogger.Log("ClimberSpeed", climberSpeed);
        HotLogger.Log("ClimberCurrent", climberMotor.getStatorCurrent());
        SmartDashboard.putNumber("actualPosTicks", climberMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("actualPosDeg", actualPosDeg);
        SmartDashboard.putString("climberState", climberState.toString());
        HotLogger.Log("climberState", climberState.toString());
        HotLogger.Log("targetPosDeg", targetPosDeg);
        HotLogger.Log("targetPosTicks", targetPosTicks);
        HotLogger.Log("actualPosTicks", climberMotor.getSelectedSensorPosition());
        HotLogger.Log("actualPosDeg", actualPosDeg);
        SmartDashboard.putBoolean("manualControlFlag", manualControlFlag);
        if (climberExtend.get() == Value.kForward){
            SmartDashboard.putString("Climber Direction", "kforward");
        }
        else if (climberExtend.get() == Value.kReverse){
            SmartDashboard.putString("Climber Direction", "kreverse");
        }
        else if (climberExtend.get() == Value.kOff){
            SmartDashboard.putString("Climber Direction", "koff");
        }

    }

    private double degreeToTicks(double degrees){
        return ((347.222 * degrees) / 360) * 2048;
    }
    private double ticksToDegrees(double ticks){
        return ((ticks / 2048) * 360) / 347.222;
    }
}