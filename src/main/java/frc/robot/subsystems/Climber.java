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
        climberMotor.setSelectedSensorPosition((PACKAGE_ANGLE * CLIMBER_TICKS_PER_DEGREE), 0, 30);
    }

    @Override
    public void enabledAction(RobotState robotState, RobotCommander commander) {

        if (commander.getClimberExtend()) {
            robotState.setClimberExtended(true);
            climberExtend.set(DoubleSolenoid.Value.kReverse);
        }
        
        if (commander.getClimberRetract()) {
            robotState.setClimberExtended(false);
            climberExtend.set(DoubleSolenoid.Value.kForward);
        }
        targetPosDeg = commander.getClimberAngle();
        targetPosTicks = (commander.getClimberAngle() * CLIMBER_TICKS_PER_DEGREE);
        climberMotor.set(TalonFXControlMode.MotionMagic, targetPosTicks);
        //climberMotor.set(TalonFXControlMode.PercentOutput, commander.getClimberMotor());
        climberRelease.set(commander.getClimberRelease());
        actualPosTicks = climberMotor.getSelectedSensorPosition();
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
    }
}
