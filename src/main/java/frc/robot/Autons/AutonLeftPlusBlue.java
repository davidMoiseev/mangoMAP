package frc.robot.Autons;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import com.revrobotics.SparkMaxRelativeEncoder;

import edu.wpi.first.hal.HAL.SimPeriodicAfterCallback;
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
import frc.robot.Autons.AutonRight5Ball.AutoState;
import frc.robot.sensors.Pigeon;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter.Shot;

public class AutonLeftPlusBlue extends AutonCommader {

    public static final String name = "LEFT";

    enum AutoState {
        prepareToShootInitialBall,
        shootInitialBall,
        driveToBall,
        pickUpBall,
        autoComplete, 
        finalShot,
        startTimer,
        getBlue,
        shootBlue,
        turnToTarget,
        pauseAtBall,
        pauseAtBlue,
        turnToHanger,
        finalTurn
    }

    private AutoState autoState; 


    private Trajectory trajectoryStartToFirstBall;
    private boolean autonInProgress;


    private Timer timer;


    private State lastDesiredState;


    private Trajectory trajectoryStartToSecondBall;


    private Shot hoodPosition = Shot.NEUTRAL;


    private boolean driveRequested;


    private boolean shoot = false;


    private boolean deployRightIntake = false;

    private State startState;


    private boolean deployLeftIntake = false;


    private boolean autoAim;

    public AutonLeftPlusBlue(RobotState robotState) {
        super(robotState);
        autoState = AutoState.startTimer;
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
        return deployLeftIntake;
    }

    @Override
    public boolean getRunRightIntake() {
        // TODO Auto-generated method stub
        return deployRightIntake;
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
    public boolean getResetIMU() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean getRobotAim() {
        return autoAim;
    }

    @Override
    public Shot getHoodPosition() {
        // TODO Auto-generated method stub
        return hoodPosition;
    }

    @Override
    public boolean[] getBallivator() {
        boolean RT= false, LT = false, dRT = false, enable = false, stop = !shoot;
        if(deployRightIntake){
          RT= true;
        } else {
          RT= false;
        }
        if(deployLeftIntake){
          LT = true;
        } else {
          LT = false;
        }
        if(shoot){
          dRT = true;
        } else {
          dRT = false;
        }
        boolean[] tmp = {RT, LT, enable, dRT, stop};
        return tmp;
    }




    @Override
    public Pose2d getInitialPose() {
        return lastDesiredState.poseMeters;
    }




    @Override
    public void initializeAuton() {
        lastDesiredState = new State(0.0, 0.0, 0.0, new Pose2d(0, 0,new Rotation2d(Math.toRadians(315))), 
        1000);

        startState = lastDesiredState;
    }


    public boolean getAutonInProgress() {
        return autonInProgress;
    }

    @Override
    public void updateCommand(){

    }

    public void updateCommand(Pigeon pigeon, Drivetrain drivetrain) {
        if(autoState == AutoState.startTimer){
            autoState = AutoState.prepareToShootInitialBall;
            timer.reset();
            timer.start();
        }
        if (autoState == AutoState.prepareToShootInitialBall) {
            autonInProgress = true;
            hoodPosition = Shot.WALL;
            pigeon.initializeAuton(this);
            autoAim = false;
            if(timer.get() > .25) {
                autoState = AutoState.shootInitialBall;
                timer.reset();
                timer.start();
            }
        }
        if (autoState == AutoState.shootInitialBall) {
            autonInProgress = true;
            driveRequested = false;
            hoodPosition = Shot.WALL;
            // shoot = true;
            autoAim = false;
            
            if(timer.get() > .25) { // .5
                timer.reset();
                timer.start();
                drivetrain.initializeAuton(this);
                autoState = AutoState.driveToBall;
            }
        }

        if (autoState == AutoState.driveToBall){
            autonInProgress = true;
            driveRequested = true;
            deployRightIntake = true;
            deployLeftIntake = false;
            hoodPosition = Shot.WALL;
            autoAim = false;
            shoot = false;
            desiredState = new State(timer.get(), .6/2*timer.get(), .6/2, new Pose2d(lastDesiredState.poseMeters.getX()-1.75/2*timer.get(),
                                                                    lastDesiredState.poseMeters.getY()+.75/2*timer.get(),
                                                                    lastDesiredState.poseMeters.getRotation()), 
                                                                    1000);
            
            setTargetTheta(Rotation2d.fromDegrees(275));
            
            SmartDashboard.putNumber("TargetX", desiredState.poseMeters.getTranslation().getX());
            SmartDashboard.putNumber("TargetY", desiredState.poseMeters.getTranslation().getY());
            SmartDashboard.putNumber("TargetTheta", desiredState.poseMeters.getRotation().getDegrees());

            if (timer.get() > 2) {
                lastDesiredState = desiredState;
                autoState = AutoState.pauseAtBall;
                timer.reset();
                timer.start();
            }                  
        }
        if(autoState == AutoState.pauseAtBall){
            autonInProgress = true;
            driveRequested = true;
            deployRightIntake = true;
            deployLeftIntake = false;
            hoodPosition = Shot.WALL;
            autoAim = false;
            shoot = false;

            setTargetTheta(Rotation2d.fromDegrees(275));

            if (timer.get() > 1) {
                lastDesiredState = desiredState;
                autoState = AutoState.turnToTarget;
                timer.reset();
                timer.start();
            }   
        }
        if (autoState == AutoState.turnToTarget) {
            autonInProgress = true;
            driveRequested = true;
            deployRightIntake = false;
            deployLeftIntake = true;
            hoodPosition = Shot.WALL;
            autoAim = false;

            desiredState = new State(timer.get(), .6/1.5*timer.get(), .6/1.5, new Pose2d(lastDesiredState.poseMeters.getX()+.5/1.5*timer.get(),
                                    lastDesiredState.poseMeters.getY()-.08/1.5*timer.get(),
                                    lastDesiredState.poseMeters.getRotation()), 
                                    1000);

            setTargetTheta(Rotation2d.fromDegrees(320));

            SmartDashboard.putNumber("TargetX", desiredState.poseMeters.getTranslation().getX());
            SmartDashboard.putNumber("TargetY", desiredState.poseMeters.getTranslation().getY());
            SmartDashboard.putNumber("TargetTheta", desiredState.poseMeters.getRotation().getDegrees());
            if (timer.get() > 1.5) {
                autoState = AutoState.finalShot;
                lastDesiredState = desiredState;
                timer.reset();
                timer.start();
            }              
        }
        if(autoState == AutoState.finalShot){
            autonInProgress = true;
            driveRequested = false;
            deployLeftIntake = true;
            autoAim = false;
            hoodPosition = Shot.WALL;
            shoot = true;
            
            if(timer.get() > 3) {
                timer.reset();
                timer.start();
                autoState = AutoState.getBlue;        
            }
        }
        if(autoState == AutoState.getBlue){
            autonInProgress = true;
            driveRequested = true;
            deployRightIntake = false;
            deployLeftIntake = true;
            hoodPosition = Shot.FENDER;
            autoAim = false;
            shoot = false;

            desiredState = new State(timer.get(), .6/1*timer.get(), .6/1, new Pose2d(lastDesiredState.poseMeters.getX()+.75/1*timer.get(),
                                    lastDesiredState.poseMeters.getY()+.8/1*timer.get(),
                                    lastDesiredState.poseMeters.getRotation()), 
                                    1000);

            setTargetTheta(Rotation2d.fromDegrees(320));


            if(timer.get() > 1) {
                timer.reset();
                timer.start();
                autoState = AutoState.pauseAtBlue;        
            }
        }
        if(autoState == AutoState.pauseAtBlue){
            autonInProgress = true;
            driveRequested = true;
            deployRightIntake = false;
            deployLeftIntake = true;
            hoodPosition = Shot.CHAOS;
            autoAim = false;
            shoot = false;

            setTargetTheta(Rotation2d.fromDegrees(320));

            if (timer.get() > 1) {
                lastDesiredState = desiredState;
                autoState = AutoState.turnToHanger;
                timer.reset();
                timer.start();
            }   
        }
        if(autoState == AutoState.turnToHanger){
            autonInProgress = true;
            driveRequested = true;
            deployRightIntake = false;
            deployLeftIntake = false;
            hoodPosition = Shot.CHAOS;
            autoAim = false;
            shoot = false;

            setTargetTheta(Rotation2d.fromDegrees(185));

            if(timer.get() > 2.5) {
                timer.reset();
                timer.start();
                autoState = AutoState.shootBlue;        
            }
        }
        if(autoState == AutoState.shootBlue){
            autonInProgress = true;
            driveRequested = true;
            deployRightIntake = false;
            deployLeftIntake = false;
            hoodPosition = Shot.CHAOS;
            autoAim = false;
            shoot = true;

            setTargetTheta(Rotation2d.fromDegrees(185));

            if (timer.get() > 1) {
                lastDesiredState = desiredState;
                autoState = AutoState.finalTurn;
                timer.reset();
                timer.start();
            }   
        }
        if(autoState == AutoState.finalTurn){
            autonInProgress = true;
            driveRequested = true;
            deployRightIntake = false;
            deployLeftIntake = false;
            hoodPosition = Shot.FENDER;
            autoAim = false;
            shoot = false;

            desiredState = new State(timer.get(), .6/1*timer.get(), .6/1, new Pose2d(lastDesiredState.poseMeters.getX()+.08/1*timer.get(),
            lastDesiredState.poseMeters.getY()+.7/1*timer.get(),
            lastDesiredState.poseMeters.getRotation()), 
            1000);

            setTargetTheta(Rotation2d.fromDegrees(270));

            if (timer.get() > 1) {
                lastDesiredState = desiredState;
                autoState = AutoState.autoComplete;
                timer.reset();
                timer.start();
            }   
        }
        if (autoState == AutoState.autoComplete) {
            deployRightIntake = false;
            deployLeftIntake = false;
            autoAim = false;
            autonInProgress = false;
            driveRequested = false;
            hoodPosition = Shot.NEUTRAL;
        }
       
        SmartDashboard.putString("AutoState", ""+autoState);

        SmartDashboard.putNumber("Start Pose X", startState.poseMeters.getX());
        SmartDashboard.putNumber("Start Pose Y", startState.poseMeters.getY());
        
    }




    @Override
    public boolean getOverrideShooterMotor() {
        // TODO Auto-generated method stub
        return false;
    }




    @Override
    public boolean getOverrideBallivatorMotor() {
        // TODO Auto-generated method stub
        return false;
    }




    @Override
    public boolean getOverrideIntakmotor() {
        // TODO Auto-generated method stub
        return false;
    }




    @Override
    public boolean getDriveRequested() {
        // TODO Auto-generated method stub
        return driveRequested;
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
    public boolean getAutoAimSetTarget() {
        // TODO Auto-generated method stub
        return false;
    }
}
