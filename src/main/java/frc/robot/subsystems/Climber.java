package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import org.hotutilites.hotlogger.HotLogger;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PneumaticHub;
import frc.robot.RobotCommander;
import frc.robot.RobotState;
import static frc.robot.Constants.*;

public class Climber extends SubsystemBase{
    private TalonFX climberMotor;
    private DoubleSolenoid climberExtend;
    private Solenoid climberRelease;
    RobotState robotState;

    public Climber(RobotState robotState, PneumaticHub hub) {
        this.robotState = robotState;
        climberMotor = new TalonFX(CLIMBER_MOTOR);
        climberExtend = hub.makeDoubleSolenoid(CLIMBER_EXTEND_FWD_SOLENOID, CLIMBER_EXTEND_REV_SOLENOID);
        climberRelease = hub.makeSolenoid(CLIMBER_RELEASE_SOLENOID);
    }

    @Override
    public void enabledAction(RobotState robotState, RobotCommander commander) {

        // Move Climber motor back into if statement later
        if (commander.getClimberExtend()){
            climberExtend.set(DoubleSolenoid.Value.kReverse);
        }
        climberMotor.set(TalonFXControlMode.PercentOutput, commander.getClimberMotor());
        climberRelease.set(commander.getClimberRelease());
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
    }
}
