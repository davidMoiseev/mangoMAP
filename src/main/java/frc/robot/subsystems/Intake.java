package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.PneumaticHub;
import frc.robot.RobotCommander;
import frc.robot.RobotState;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import org.hotutilites.hotlogger.HotLogger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import static frc.robot.Constants.*;

import javax.rmi.ssl.SslRMIClientSocketFactory;

public class Intake extends SubsystemBase {
    RobotState robotState;
    Drivetrain drivetrain;
    TalonFX rightIntakeMotor;
    TalonFX leftIntakeMotor;
    DoubleSolenoid rightIntakeSolenoid;
    DoubleSolenoid leftIntakeSolenoid;
    boolean runLeftIntake;
    boolean runRightIntake;

    double driveDir;
    double theta;

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));

    public Intake(RobotState robotState, PneumaticHub hub, Drivetrain m_drivetrain){
        this.robotState = robotState;

        rightIntakeMotor = new TalonFX(RIGHT_INTAKE_MOTOR);
        leftIntakeMotor = new TalonFX(LEFT_INTAKE_MOTOR);
        rightIntakeSolenoid = hub.makeDoubleSolenoid(RIGHT_INTAKE_FWD_SOLENOID, RIGHT_INTAKE_REV_SOLENOID);
        leftIntakeSolenoid = hub.makeDoubleSolenoid(LEFT_INTAKE_FWD_SOLENOID, LEFT_INTAKE_REV_SOLENOID);
        rightIntakeMotor.setNeutralMode((NeutralMode.Coast));
        leftIntakeMotor.setNeutralMode((NeutralMode.Coast));

        drivetrain = m_drivetrain;
    }
    
    boolean intakeState = false;

    @Override
    public void enabledAction(RobotState robotState, RobotCommander commander) {
        if(COMP_BOT){
            if(commander.getAutoIntakeDeploy()){
                ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    commander.getForwardCommand(), commander.getStrafeCommand(),
                    0, Rotation2d.fromDegrees(robotState.getTheta()));

                if(chassisSpeeds.vyMetersPerSecond > .05){
                    runRightIntake = false;
                    rightIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
                    rightIntakeMotor.set(ControlMode.PercentOutput, 0.0);
                    
                    intakeState = true;
                    
                    runLeftIntake = true;
                    leftIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
                    leftIntakeMotor.set(ControlMode.PercentOutput, INTAKE_POWER);
                } else if (chassisSpeeds.vyMetersPerSecond < -.05){
                    runLeftIntake = false;
                    leftIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
                    leftIntakeMotor.set(ControlMode.PercentOutput, 0.0);

                    intakeState = true;

                    runRightIntake = true;
                    rightIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
                    rightIntakeMotor.set(ControlMode.PercentOutput, (-1 * INTAKE_POWER));
                } else {
                    
                    runLeftIntake = false;
                    leftIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
                    leftIntakeMotor.set(ControlMode.PercentOutput, 0.0);

                    intakeState = false;

                    runRightIntake = false;
                    rightIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
                    rightIntakeMotor.set(ControlMode.PercentOutput, 0.0);
                }

            } else {
                if (commander.getRunLeftIntake()) {
                    runLeftIntake = true;
                    leftIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
                    leftIntakeMotor.set(ControlMode.PercentOutput, INTAKE_POWER);
                } else {
                    runLeftIntake = false;
                    leftIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
                    leftIntakeMotor.set(ControlMode.PercentOutput, 0.0);
                } 

                if (commander.getRunRightIntake()) {
                    runRightIntake = true;
                    rightIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
                    rightIntakeMotor.set(ControlMode.PercentOutput, (-1 * INTAKE_POWER));
                } else {
                    runRightIntake = false;
                    rightIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
                    rightIntakeMotor.set(ControlMode.PercentOutput, 0.0);
                }
                if (commander.getRunLeftIntake() || commander.getRunRightIntake()) {
                    intakeState = true;
                } else {
                    intakeState = false;
                }
            }
        } else {
            if (commander.getRunLeftIntake()) {
                runLeftIntake = true;
                leftIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
                leftIntakeMotor.set(ControlMode.PercentOutput, INTAKE_POWER);
            } else {
                runLeftIntake = false;
                leftIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
                leftIntakeMotor.set(ControlMode.PercentOutput, 0.0);
            }

            if (commander.getRunRightIntake()) {
                runRightIntake = true;
                rightIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
                rightIntakeMotor.set(ControlMode.PercentOutput, (-1 * INTAKE_POWER));
            } else {
                runRightIntake = false;
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
        SmartDashboard.putBoolean("runRightIntake", runRightIntake); 
        SmartDashboard.putBoolean("runLeftIntake", runLeftIntake); 
    }
    public boolean getIntakeState(){
        return intakeState;
    }
}
