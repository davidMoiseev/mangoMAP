package frc.robot.Autons;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.AutonCommader;
import frc.robot.Constants;
import frc.robot.RobotState;

public class AutonRight extends AutonCommader {

    public static final String name = "RIGHT";

    enum AutoState {
        prepareToShootInitialBall,
        shootInitialBall,
        driveToWall,
        pickUpBall,
        pathToSecondBall,
        autoComplete
    }

    private AutoState autoState; 


    private Trajectory trajectoryStartToFirstBall;
    private boolean autonInProgress;


    private Timer timer;


    private State lastDesiredState;


    private Trajectory trajectoryStartToSecondBall;

    public AutonRight(RobotState robotState) {
        super(robotState);
        autoState = AutoState.prepareToShootInitialBall;
        timer = new Timer();

        trajectoryStartToFirstBall = readTrajectoryFile(AutonRightConstants.trajectoryJSON_FileName_ToWall);
        trajectoryStartToSecondBall = readTrajectoryFile(AutonRightConstants.trajectoryJSON_FileNameToSecondBall);         

        // SwerveDriveKinematicsConstraint swerveConstraint = new SwerveDriveKinematicsConstraint(Constants.KINEMATICS, 4.2672);

        // TrajectoryConfig config = new TrajectoryConfig(4.2672, 4.2672)
        // .setKinematics(Constants.KINEMATICS)
        // .addConstraint(swerveConstraint);    

        // setTrajectory(TrajectoryGenerator.generateTrajectory(List.of(
        //                 new Pose2d(9.836, 6.3, new Rotation2d(Math.toRadians(36.812))),
        //                  new Pose2d(13.8,6.3,new Rotation2d(Math.toRadians(90)))), 
        //                  config));

        // setTrajectory(TrajectoryGenerator.generateTrajectory(new Pose2d(9.836, 6.3, new Rotation2d(Math.toRadians(36.812))), 
        // List.of(new Translation2d(10,6.3)), 
        // new Pose2d(11.8,6.3,new Rotation2d(Math.toRadians(36.812))), 
        // config));
    }


    

    @Override
    public String getName() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public double getForwardCommand() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getStrafeCommand() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getTurnCommand() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public boolean getRunLeftIntake() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean getRunRightIntake() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean getClimberChangeState() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public double getClimberMotor() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public boolean getClimberRelease() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public double getRightIntakeCommand() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getLeftIntakeCommand() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public boolean getResetIMU() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean getRobotAim() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public int getHoodPosition() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getShooterSpeed() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public boolean[] getBallivator() {
        // TODO Auto-generated method stub
        return null;
    }




    @Override
    public Pose2d getInitialPose() {
        return trajectoryStartToFirstBall.getInitialPose();
    }




    @Override
    public void initializeAuton() {
        
    }


    public boolean getAutonInProgress() {
        return autonInProgress;
    }

    @Override
    public void updateCommand() {
        if (autoState == AutoState.prepareToShootInitialBall) {
            autonInProgress = true;
            if(true) {
                autoState = AutoState.shootInitialBall;
            }
        }
        if (autoState == AutoState.shootInitialBall) {
            autonInProgress = true;
            if(true) {
                timer.reset();
                timer.start();
                autoState = AutoState.driveToWall;
            }
        }
        if (autoState == AutoState.driveToWall) {
            autonInProgress = true;
            desiredState = trajectoryStartToFirstBall.sample(timer.get()); 
            setTargetTheta(trajectoryStartToFirstBall.sample(trajectoryStartToFirstBall.getTotalTimeSeconds()).poseMeters.getRotation());
            
            SmartDashboard.putNumber("TargetX", desiredState.poseMeters.getTranslation().getX());
            SmartDashboard.putNumber("TargetY", desiredState.poseMeters.getTranslation().getY());
            SmartDashboard.putNumber("TargetTheta", desiredState.poseMeters.getRotation().getDegrees());
            if (timer.get() > trajectoryStartToFirstBall.getTotalTimeSeconds()-.1) {
                autoState = AutoState.pickUpBall;
                lastDesiredState = desiredState;
                timer.reset();
                timer.start();
            }
        }
        if (autoState == AutoState.pickUpBall) {
            autonInProgress = true;
            desiredState = new State(timer.get(), 1/.2*timer.get(), .75, new Pose2d(lastDesiredState.poseMeters.getX()-.5/.2*timer.get(),
                                                                    lastDesiredState.poseMeters.getY(),
                                                                    lastDesiredState.poseMeters.getRotation()), 
                                                                    1000);
            setTargetTheta(lastDesiredState.poseMeters.getRotation());    
            if (timer.get() > .3) {
                autoState = AutoState.pathToSecondBall;
                timer.reset();
                timer.start();
            }                  
        }
        if (autoState == AutoState.pathToSecondBall) {
            autonInProgress = true;
            desiredState = trajectoryStartToSecondBall.sample(timer.get()); 
            setTargetTheta(trajectoryStartToSecondBall.sample(trajectoryStartToFirstBall.getTotalTimeSeconds()).poseMeters.getRotation());
            
            SmartDashboard.putNumber("TargetX", desiredState.poseMeters.getTranslation().getX());
            SmartDashboard.putNumber("TargetY", desiredState.poseMeters.getTranslation().getY());
            SmartDashboard.putNumber("TargetTheta", desiredState.poseMeters.getRotation().getDegrees());
            if (timer.get() > trajectoryStartToSecondBall.getTotalTimeSeconds()-.1) {
                autoState = AutoState.autoComplete;
                lastDesiredState = desiredState;
                timer.reset();
                timer.start();
            }
        }
        if (autoState == AutoState.autoComplete) {
            autonInProgress = false;
        }
        
    }
}
