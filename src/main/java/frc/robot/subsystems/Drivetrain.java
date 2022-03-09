package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import org.hotutilites.hotlogger.HotLogger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.AutonCommader;
import frc.robot.Constants;
import frc.robot.RobotCommander;
import frc.robot.RobotState;

import static frc.robot.Constants.*;

public class Drivetrain extends SubsystemBase {

        private final SwerveModule frontLeftModule;
        private final SwerveModule frontRightModule;
        private final SwerveModule backLeftModule;
        private final SwerveModule backRightModule;

        private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

        private SwerveModuleState[] states;

        private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));

        private double yOffset = 0;
        private HolonomicDriveController holonomicController;
        private SwerveDrivePoseEstimator poseExstimator;
        private RobotState robotState;

        private TalonFX leftFrontDrive;
        private TalonFX rightFrontDrive;
        private TalonFX leftBackDrive;
        private TalonFX rightBackDrive;

        private TalonFX leftFrontSteer;
        private TalonFX rightFrontSteer;
        private TalonFX leftBackSteer;
        private TalonFX rightBackSteer;
        boolean firstLoop;
        double targetY;


        public Drivetrain(RobotState robotState) {
                this.robotState = robotState;
                ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

                if (REAL_BOT) {
                        frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
                                        tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                                                        .withSize(2, 4)
                                                        .withPosition(0, 0),
                                        Mk4iSwerveModuleHelper.GearRatio.L2,
                                        FRONT_LEFT_MODULE_DRIVE_MOTOR,
                                        FRONT_LEFT_MODULE_STEER_MOTOR,
                                        FRONT_LEFT_MODULE_STEER_ENCODER,
                                        FRONT_LEFT_MODULE_STEER_OFFSET);

                        frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(
                                        tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                                                        .withSize(2, 4)
                                                        .withPosition(2, 0),
                                        Mk4iSwerveModuleHelper.GearRatio.L2,
                                        FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                                        FRONT_RIGHT_MODULE_STEER_MOTOR,
                                        FRONT_RIGHT_MODULE_STEER_ENCODER,
                                        FRONT_RIGHT_MODULE_STEER_OFFSET);

                        backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
                                        tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                                                        .withSize(2, 4)
                                                        .withPosition(4, 0),
                                        Mk4iSwerveModuleHelper.GearRatio.L2,
                                        BACK_LEFT_MODULE_DRIVE_MOTOR,
                                        BACK_LEFT_MODULE_STEER_MOTOR,
                                        BACK_LEFT_MODULE_STEER_ENCODER,
                                        BACK_LEFT_MODULE_STEER_OFFSET);

                        backRightModule = Mk4iSwerveModuleHelper.createFalcon500(
                                        tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                                                        .withSize(2, 4)
                                                        .withPosition(6, 0),
                                        Mk4iSwerveModuleHelper.GearRatio.L2,
                                        BACK_RIGHT_MODULE_DRIVE_MOTOR,
                                        BACK_RIGHT_MODULE_STEER_MOTOR,
                                        BACK_RIGHT_MODULE_STEER_ENCODER,
                                        BACK_RIGHT_MODULE_STEER_OFFSET);
                } else {
                        frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                                        tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                                                        .withSize(2, 4)
                                                        .withPosition(0, 0),
                                        Mk4SwerveModuleHelper.GearRatio.L2,
                                        FRONT_LEFT_MODULE_DRIVE_MOTOR,
                                        FRONT_LEFT_MODULE_STEER_MOTOR,
                                        FRONT_LEFT_MODULE_STEER_ENCODER,
                                        FRONT_LEFT_MODULE_STEER_OFFSET);

                        frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
                                        tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                                                        .withSize(2, 4)
                                                        .withPosition(2, 0),
                                        Mk4SwerveModuleHelper.GearRatio.L2,
                                        FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                                        FRONT_RIGHT_MODULE_STEER_MOTOR,
                                        FRONT_RIGHT_MODULE_STEER_ENCODER,
                                        FRONT_RIGHT_MODULE_STEER_OFFSET);

                        backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                                        tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                                                        .withSize(2, 4)
                                                        .withPosition(4, 0),
                                        Mk4SwerveModuleHelper.GearRatio.L2,
                                        BACK_LEFT_MODULE_DRIVE_MOTOR,
                                        BACK_LEFT_MODULE_STEER_MOTOR,
                                        BACK_LEFT_MODULE_STEER_ENCODER,
                                        BACK_LEFT_MODULE_STEER_OFFSET);

                        backRightModule = Mk4SwerveModuleHelper.createFalcon500(
                                        tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                                                        .withSize(2, 4)
                                                        .withPosition(6, 0),
                                        Mk4SwerveModuleHelper.GearRatio.L2,
                                        BACK_RIGHT_MODULE_DRIVE_MOTOR,
                                        BACK_RIGHT_MODULE_STEER_MOTOR,
                                        BACK_RIGHT_MODULE_STEER_ENCODER,
                                        BACK_RIGHT_MODULE_STEER_OFFSET);
                }
                poseExstimator = new SwerveDrivePoseEstimator(
                        new Rotation2d(),
                        new Pose2d(),
                        Constants.KINEMATICS,
                        VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
                        VecBuilder.fill(Units.degreesToRadians(0.01)),
                        VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

                states = kinematics.toSwerveModuleStates(new ChassisSpeeds());

                // leftFrontDrive = new TalonFX(FRONT_LEFT_MODULE_DRIVE_MOTOR);
                // rightFrontDrive = new TalonFX(FRONT_RIGHT_MODULE_DRIVE_MOTOR);
                // leftBackDrive = new TalonFX(BACK_LEFT_MODULE_DRIVE_MOTOR);
                // rightBackDrive = new TalonFX(FRONT_LEFT_MODULE_DRIVE_MOTOR);

                // leftFrontSteer = new TalonFX(FRONT_LEFT_MODULE_STEER_MOTOR);
                // rightFrontSteer = new TalonFX(FRONT_RIGHT_MODULE_STEER_MOTOR);
                // leftBackSteer = new TalonFX(BACK_LEFT_MODULE_STEER_MOTOR);
                // rightBackSteer = new TalonFX(FRONT_LEFT_MODULE_STEER_MOTOR);
        }

        private void setSwerveModuleStates(ChassisSpeeds chassisSpeeds) {
                states = kinematics.toSwerveModuleStates(chassisSpeeds);
                SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

                frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[0].angle.getRadians());
                frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[1].angle.getRadians());
                backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[2].angle.getRadians());
                backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[3].angle.getRadians());
        }

        public void initializeAuton (AutonCommader commander){
                poseExstimator.resetPosition(commander.getInitialPose(), robotState.getRotation2d());

                ProfiledPIDController thetaController = new ProfiledPIDController(6.5, 1.4, .09,  // Theta
                                                        new TrapezoidProfile.Constraints(6.28, 3.14));

                thetaController.enableContinuousInput(-Math.PI, Math.PI);
                holonomicController = new HolonomicDriveController(
                        new PIDController(16, 1.4, 0.2),  //x Long side of field
                        new PIDController(16, 1.4, 0.2), //y Short side of field
                        thetaController); // (2Pk,PI) constrains to 1 2pi/sec
        }

        public void autonenabledAction(AutonCommader commander) {

                if (commander.getRobotAim()) {
                        yOffset += commander.getForwardCommand() * Constants.Y_OFFSET; //makes the robot go forwards or backwards(robot centric) while turning around the hub
                         // if limelight has target 
                         if(commander.getAutonInProgress()){
                                if (robotState.getDetecting() == 1) {
                                        // auto aim
                                        SmartDashboard.putBoolean("Auton Auto Aiming", true);
                                        chassisSpeeds = new ChassisSpeeds(
                                                        0,
                                                        0,
                                                        (Math.abs(robotState.getTxReal()) > Constants.ALLOWED_X_OFFSET) ? robotState.getTxReal() * Constants.X_ADJUST_SPEED : 0);
                                }
                         } else {
                                if (robotState.getDetecting() == 1) {
                                        // auto aim
                                        chassisSpeeds = new ChassisSpeeds(
                                                        (Math.abs(robotState.getTyReal() + yOffset) > Constants.ALLOWED_Y_OFFSET) ? (robotState.getTyReal() + yOffset) * Constants.Y_ADJUST_SPEED : 0,
                                                        Constants.STRAFE_SPEED * commander.getStrafeCommand(),
                                                        (Math.abs(robotState.getTxReal()) > Constants.ALLOWED_X_OFFSET) ? robotState.getTxReal() * Constants.X_ADJUST_SPEED : 0);
                                }
                         }
                 } else if (commander.getAutonInProgress() && commander.getDriveRequested()) {
                        setSwerveModuleStates(holonomicController.calculate(poseExstimator.getEstimatedPosition(), 
                                                        commander.getDesiredState(),
                                                        commander.getTargetTheta()));
                } else {
                        setSwerveModuleStates(new ChassisSpeeds());
                }
        }

        @Override
        public void enabledAction(RobotState robotState, RobotCommander commander) {
                
                // if not pressing A
                if (!commander.getRobotAim()) {
                        firstLoop = true;
                        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                                        commander.getForwardCommand(),
                                        commander.getStrafeCommand(),
                                        commander.getTurnCommand(),
                                        robotState.getRotation2d());
                        
                        yOffset = 0;
                        // if pressing A
                } else if(commander.getAutoAimSetTarget()){
                        
                        if(firstLoop){
                                targetY = robotState.getTyReal();
                                firstLoop = false;
                        }
                        yOffset += commander.getForwardCommand() * Constants.Y_OFFSET; //makes the robot go forwards or backwards(robot centric) while turning around the hub
                        // if limelight has target 
                        if (robotState.getDetecting() == 1) {
                                // auto aim
                                chassisSpeeds = new ChassisSpeeds(
                                                (Math.abs(robotState.getTyReal() + yOffset) > targetY) ? (robotState.getTyReal() + yOffset) * Constants.Y_ADJUST_SPEED : 0,
                                                Constants.STRAFE_SPEED * commander.getStrafeCommand(),
                                                (Math.abs(robotState.getTxReal()) > Constants.ALLOWED_X_OFFSET) ? robotState.getTxReal() * Constants.X_ADJUST_SPEED : 0);
                        }
                } else {
                       firstLoop = true;
                       yOffset += commander.getForwardCommand() * Constants.Y_OFFSET; //makes the robot go forwards or backwards(robot centric) while turning around the hub
                        // if limelight has target 
                        if (robotState.getDetecting() == 1) {
                                // auto aim
                                chassisSpeeds = new ChassisSpeeds(
                                                (Math.abs(robotState.getTyReal() + yOffset) > Constants.ALLOWED_Y_OFFSET) ? (robotState.getTyReal() + yOffset) * Constants.Y_ADJUST_SPEED : 0,
                                                Constants.STRAFE_SPEED * commander.getStrafeCommand(),
                                                (Math.abs(robotState.getTxReal()) > Constants.ALLOWED_X_OFFSET) ? robotState.getTxReal() * Constants.X_ADJUST_SPEED : 0);
                        }
                }
                setSwerveModuleStates(chassisSpeeds);
        }

        @Override
        public void disabledAction(RobotState robotState) {
                // setBrakeMode(false);
        }

        @Override
        public void updateState() {
                poseExstimator.update(robotState.getRotation2d(), states);

                SmartDashboard.putNumber("Robot State Theta", robotState.getRotation2d().getDegrees());
                SmartDashboard.putNumber("poseX", poseExstimator.getEstimatedPosition().getX());
                SmartDashboard.putNumber("poseY", poseExstimator.getEstimatedPosition().getY());
                SmartDashboard.putNumber("poseTheta", poseExstimator.getEstimatedPosition().getRotation().getDegrees());
        }

        @Override
        public void zeroActuators() {
                frontLeftModule.set(0, 0);
                frontRightModule.set(0, 0);
                backLeftModule.set(0, 0);
                backRightModule.set(0, 0);
        }

        @Override
        public void zeroSensor() {

        }

        @Override
        public void logData() {

        }

        public void setBrakeMode(boolean brakes) {
                // if(brakes){
                //         leftFrontDrive.setNeutralMode(NeutralMode.Brake);
                //         rightFrontDrive.setNeutralMode(NeutralMode.Brake);
                //         leftBackDrive.setNeutralMode(NeutralMode.Brake);
                //         rightBackDrive.setNeutralMode(NeutralMode.Brake);

                //         leftFrontSteer.setNeutralMode(NeutralMode.Brake);
                //         rightFrontSteer.setNeutralMode(NeutralMode.Brake);
                //         leftBackSteer.setNeutralMode(NeutralMode.Brake);
                //         rightBackSteer.setNeutralMode(NeutralMode.Brake);
                // } else {
                //         leftFrontDrive.setNeutralMode(NeutralMode.Coast);
                //         rightFrontDrive.setNeutralMode(NeutralMode.Coast);
                //         leftBackDrive.setNeutralMode(NeutralMode.Coast);
                //         rightBackDrive.setNeutralMode(NeutralMode.Coast);

                //         leftFrontSteer.setNeutralMode(NeutralMode.Coast);
                //         rightFrontSteer.setNeutralMode(NeutralMode.Coast);
                //         leftBackSteer.setNeutralMode(NeutralMode.Coast);
                //         rightBackSteer.setNeutralMode(NeutralMode.Coast);
                // }
        }
}
