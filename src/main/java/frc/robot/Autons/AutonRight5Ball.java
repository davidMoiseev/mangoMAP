package frc.robot.Autons;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import com.revrobotics.SparkMaxRelativeEncoder;

import org.hotutilites.hotlogger.HotLogger;

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
import frc.robot.sensors.Pigeon;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter.Shot;

public class AutonRight5Ball extends AutonCommader {

    public static final String name = "RIGHT_RED";

    enum AutoState {
        prepareToShootInitialBall,
        shootInitialBall,
        driveToWall,
        pickUpBall,
        pathToSecondBall_1,
        autoComplete, pathToSecondBall_2, pauseAtWall, pauseAtBall2, driveToHuman, finalShot, pauseAtHuman, driveToFinal, backIntoHuman,
        resetDrivetrain, startTimer
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

    public AutonRight5Ball(RobotState robotState) {
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
        lastDesiredState = new State(0.0, 0.0, 0.0, new Pose2d(0,0,new Rotation2d(Math.toRadians(0))), 
        1000);

        startState = lastDesiredState;

        SmartDashboard.putNumber("Start Pose X", startState.poseMeters.getX());
        SmartDashboard.putNumber("Start Pose Y", startState.poseMeters.getY());
    }


    public boolean getAutonInProgress() {
        return autonInProgress;
    }

    @Override
    public void updateCommand(){

    }

    public void updateCommand(Pigeon pigeon, Drivetrain drivetrain) {
        // if (autoState == AutoState.prepareToShootInitialBall) {
        //     autonInProgress = true;
        //     hoodPosition = Shot.AUTO;
        //     pigeon.initializeAuton(this);
        //     autoAim = false;
        //     if(robotState.isShooterReady()) {
        //         autoState = AutoState.shootInitialBall;
        //         timer.reset();
        //         timer.start();
        //     }
        // }
        // if (autoState == AutoState.shootInitialBall) {
        //     autonInProgress = true;
        //     driveRequested = false;
        //     hoodPosition = Shot.AUTO2;
        //     shoot = true;
        //     autoAim = false;
            
        //     if(timer.get() > 1) { // .5
        //         timer.reset();
        //         timer.start();
        //         drivetrain.initializeAuton(this);
        //         autoState = AutoState.driveToWall;
        //     }
        // }
        // if (autoState == AutoState.prepareToShootInitialBall) {
        //     autonInProgress = true;
        //     autoAim = false;
        //     driveRequested = false;
        //     hoodPosition = Shot.WALL;
        //     pigeon.initializeAuton(this);
        //     autoState = AutoState.resetDrivetrain;
        // }
        // if(autoState == AutoState.resetDrivetrain){
        //     autonInProgress = true;
        //     autoAim = false;
        //     driveRequested = false;
        //     hoodPosition = Shot.WALL;
        //     drivetrain.initializeAuton(this);
        //     autoState = AutoState.driveToWall;
        //     timer.reset();
        //     timer.start();
        // }\
        if(autoState == AutoState.startTimer){
            autoState = AutoState.prepareToShootInitialBall;
            timer.reset();
            timer.start();
        }
        if (autoState == AutoState.prepareToShootInitialBall) {
            autonInProgress = true;
            hoodPosition = Shot.AUTO5B;
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
            hoodPosition = Shot.AUTO5B;
            // shoot = true;
            autoAim = false;
            
            if(timer.get() > .25) { // .5
                timer.reset();
                timer.start();
                drivetrain.initializeAuton(this);
                autoState = AutoState.driveToWall;
            }
        }
        if (autoState == AutoState.driveToWall) {
            autonInProgress = true;
            driveRequested = true;
            deployRightIntake = true;
            deployLeftIntake = false;
            hoodPosition = Shot.AUTO5B;
            autoAim = false;
            desiredState = new State(timer.get(), .6/1.5*timer.get(), .6/1.5, new Pose2d(lastDesiredState.poseMeters.getX()/1.5*timer.get(),
                                                                    lastDesiredState.poseMeters.getY()-1.15/1.5*timer.get(),
                                                                    lastDesiredState.poseMeters.getRotation()), 
                                                                    1000);
            
            setTargetTheta(Rotation2d.fromDegrees(0));
            
            SmartDashboard.putNumber("TargetX", desiredState.poseMeters.getTranslation().getX());
            SmartDashboard.putNumber("TargetY", desiredState.poseMeters.getTranslation().getY());
            SmartDashboard.putNumber("TargetTheta", desiredState.poseMeters.getRotation().getDegrees());

            if (timer.get() > 1.5) {
                lastDesiredState = desiredState;
                autoState = AutoState.pauseAtWall;
                timer.reset();
                timer.start();
            }                  
        }
        if (autoState == AutoState.pauseAtWall) {
            autonInProgress = true;
            driveRequested = true;
            deployRightIntake = true;
            deployLeftIntake = false;
            hoodPosition = Shot.AUTO5B;
            desiredState = lastDesiredState;
            setTargetTheta(Rotation2d.fromDegrees(0));
            autoAim = false;
            
            SmartDashboard.putNumber("TargetX", desiredState.poseMeters.getTranslation().getX());
            SmartDashboard.putNumber("TargetY", desiredState.poseMeters.getTranslation().getY());
            SmartDashboard.putNumber("TargetTheta", desiredState.poseMeters.getRotation().getDegrees());

            if (timer.get() > .3) {
                autoState = AutoState.pathToSecondBall_1;
                lastDesiredState = desiredState;
                timer.reset();
                timer.start();
            }                  
        }
        if (autoState == AutoState.pathToSecondBall_1) {
            autonInProgress = true;
            driveRequested = true;
            deployRightIntake = false;
            deployLeftIntake = true;
            hoodPosition = Shot.AUTO5B;
            autoAim = false;

            desiredState = new State(timer.get(), .1/1*timer.get(), .1/1, new Pose2d(lastDesiredState.poseMeters.getX()-1.5/1*timer.get(),
                                                                    lastDesiredState.poseMeters.getY()+.16/1*timer.get(),
                                                                    lastDesiredState.poseMeters.getRotation()), 
                                                                    1000);
            setTargetTheta(Rotation2d.fromDegrees(40));

            SmartDashboard.putNumber("TargetX", desiredState.poseMeters.getTranslation().getX());
            SmartDashboard.putNumber("TargetY", desiredState.poseMeters.getTranslation().getY());
            SmartDashboard.putNumber("TargetTheta", desiredState.poseMeters.getRotation().getDegrees());
            if (timer.get() > 1) {
                autoState = AutoState.pathToSecondBall_2;
                lastDesiredState = desiredState;
                timer.reset();
                timer.start();
            }
        }       
        if (autoState == AutoState.pathToSecondBall_2) {
            autonInProgress = true;
            driveRequested = true;
            deployRightIntake = false;
            deployLeftIntake = true;
            autoAim = false;
            hoodPosition = Shot.AUTO5B;
            shoot = true;

            desiredState = new State(timer.get(), .075/1*timer.get(),  .075/1, new Pose2d(lastDesiredState.poseMeters.getX()-.75/1*timer.get(),
                                                                    lastDesiredState.poseMeters.getY()+.7/1*timer.get(),
                                                                    lastDesiredState.poseMeters.getRotation()), 
                                                                    1000);

            setTargetTheta(Rotation2d.fromDegrees(35));

            SmartDashboard.putNumber("TargetX", desiredState.poseMeters.getTranslation().getX());
            SmartDashboard.putNumber("TargetY", desiredState.poseMeters.getTranslation().getY());
            SmartDashboard.putNumber("TargetTheta", desiredState.poseMeters.getRotation().getDegrees());
            if (timer.get() > 1.5) {
                autoState = AutoState.pauseAtBall2;
                lastDesiredState = desiredState;
                timer.reset();
                timer.start();
            }
        }      
        if (autoState == AutoState.pauseAtBall2) {
            autonInProgress = true;
            driveRequested = true;
            deployRightIntake = false;
            hoodPosition = Shot.AUTO5B;
            shoot = true;
            desiredState = lastDesiredState;
            setTargetTheta(Rotation2d.fromDegrees(35));
            autoAim = false;
            
            if(timer.get() > .25){
                deployLeftIntake = false;
            } else {
                deployLeftIntake = true;
            }

            SmartDashboard.putNumber("TargetX", desiredState.poseMeters.getTranslation().getX());
            SmartDashboard.putNumber("TargetY", desiredState.poseMeters.getTranslation().getY());
            SmartDashboard.putNumber("TargetTheta", desiredState.poseMeters.getRotation().getDegrees());

            if (timer.get() > 2) {
                autoState = AutoState.driveToHuman;
                lastDesiredState = desiredState;
                timer.reset();
                timer.start();
            }                  
        }
        if (autoState == AutoState.driveToHuman) {
            autonInProgress = true;
            driveRequested = true;
            deployRightIntake = true;
            deployLeftIntake = false;
            hoodPosition = Shot.AUTO5B;
            autoAim = false;

            desiredState = new State(timer.get(), .075/1.5*timer.get(),  .075/1.5, new Pose2d(lastDesiredState.poseMeters.getX()-3.5/1.5*timer.get(),
                                                                    lastDesiredState.poseMeters.getY()/1.5*timer.get(),
                                                                    lastDesiredState.poseMeters.getRotation()), 
                                                                    1000);
            setTargetTheta(Rotation2d.fromDegrees(315));

            SmartDashboard.putNumber("TargetX", desiredState.poseMeters.getTranslation().getX());
            SmartDashboard.putNumber("TargetY", desiredState.poseMeters.getTranslation().getY());
            SmartDashboard.putNumber("TargetTheta", desiredState.poseMeters.getRotation().getDegrees());
            if (timer.get() > 1.5) {
                autoState = AutoState.backIntoHuman;
                lastDesiredState = desiredState;
                timer.reset();
                timer.start();
            }              
        }
        if(autoState == AutoState.backIntoHuman){
            autonInProgress = true;
            driveRequested = true;
            deployRightIntake = true;
            deployLeftIntake = false;
            hoodPosition = Shot.PROTECTED;
            autoAim = false;

            desiredState = new State(timer.get(), .075/.5*timer.get(),  .075/.5, new Pose2d(lastDesiredState.poseMeters.getX()-.5/.5*timer.get(),
                                                                    lastDesiredState.poseMeters.getY()-.2/.5*timer.get(),
                                                                    lastDesiredState.poseMeters.getRotation()), 
                                                                    1000);
            setTargetTheta(Rotation2d.fromDegrees(315));

            SmartDashboard.putNumber("TargetX", desiredState.poseMeters.getTranslation().getX());
            SmartDashboard.putNumber("TargetY", desiredState.poseMeters.getTranslation().getY());
            SmartDashboard.putNumber("TargetTheta", desiredState.poseMeters.getRotation().getDegrees());
            if (timer.get() > .5) {
                autoState = AutoState.pauseAtHuman;
                lastDesiredState = desiredState;
                timer.reset();
                timer.start();
            }    
        }
        if (autoState == AutoState.pauseAtHuman) {
            autonInProgress = true;
            driveRequested = true;
            deployRightIntake = true;
            deployLeftIntake = false;
            hoodPosition = Shot.PROTECTED;
            shoot = false;
            desiredState = lastDesiredState;
            setTargetTheta(Rotation2d.fromDegrees(315));
            autoAim = false;
            
            SmartDashboard.putNumber("TargetX", desiredState.poseMeters.getTranslation().getX());
            SmartDashboard.putNumber("TargetY", desiredState.poseMeters.getTranslation().getY());
            SmartDashboard.putNumber("TargetTheta", desiredState.poseMeters.getRotation().getDegrees());

            if (timer.get() > 1.5) {
                autoState = AutoState.driveToFinal;
                lastDesiredState = desiredState;
                timer.reset();
                timer.start();
            }            
        }      
        if (autoState == AutoState.driveToFinal) {
            autonInProgress = true;
            driveRequested = true;
            deployLeftIntake = false;
            hoodPosition = Shot.PROTECTED;
            autoAim = false;

            if(timer.get() > .5){
                deployRightIntake = true;
            } else {
                deployRightIntake = false;
            }

            desiredState = new State(timer.get(), .075/1.5*timer.get(),  .075/1.5, new Pose2d(lastDesiredState.poseMeters.getX()+3/1.5*timer.get(),
                                                                    lastDesiredState.poseMeters.getY()/1.5*timer.get(),
                                                                    lastDesiredState.poseMeters.getRotation()), 
                                                                    1000);
            setTargetTheta(Rotation2d.fromDegrees(24));

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
            hoodPosition = Shot.PROTECTED;
            autoAim = false;
            if(timer.get() > .25){
                shoot = true;
            }
            
            if(timer.get() > 3) {
                timer.reset();
                timer.start();
                autoState = AutoState.autoComplete;        
            }
        }
        if (autoState == AutoState.autoComplete) {
            deployRightIntake = false;
            deployLeftIntake = false;
            autoAim = false;
            shoot = false;
            autonInProgress = false;
            driveRequested = false;
            hoodPosition = Shot.NEUTRAL;
        }
        SmartDashboard.putString("AutoState", ""+autoState);
        HotLogger.Log("AutoState", ""+autoState);

        // log();
    }


    // public void log(){
    //     HotLogger.Log("TargetX", desiredState.poseMeters.getTranslation().getX());
    //     HotLogger.Log("TargetY", desiredState.poseMeters.getTranslation().getY());
    //     HotLogger.Log("TargetTheta", desiredState.poseMeters.getRotation().getDegrees());
    // }


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




    @Override
    public boolean getAutoIntakeDeploy() {
        // TODO Auto-generated method stub
        return false;
    }




    @Override
    public boolean getYButtonHeld() {
        // TODO Auto-generated method stub
        return false;
    }
}
