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
import frc.robot.sensors.Pigeon;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter.Shot;

public class AutonRight5Ballold extends AutonCommader {

    public static final String name = "RIGHT_5";

    enum AutoState {
        start,
        getWallBall,
        driveToSecondBall_1,
        driveToSecondBall_2,
        firstShot,
        getSecondBall,
        secondShot,
        driveToHumanPlayer,
        pickUpHumanBalls,
        goToLastShot,
        finalShot,
        autoComplete,
        test
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

    public AutonRight5Ballold(RobotState robotState) {
        super(robotState);
        autoState = AutoState.start;
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
        lastDesiredState = new State(0.0, 0.0, 0.0, new Pose2d(10.2,5.93,new Rotation2d(Math.toRadians(0))), 
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
        // if (autoState == AutoState.start) {
        //     autonInProgress = true;
        //     autoAim = false;
        //     driveRequested = false;
        //     hoodPosition = Shot.NEUTRAL;
        //     pigeon.initializeAuton(this);
        //     drivetrain.initializeAuton(this);
        //     autoState = AutoState.getWallBall;
        //     timer.reset();
        //     timer.start();
        // }
        if (autoState == AutoState.start) {
            autonInProgress = true;
            hoodPosition = Shot.AUTO;
            pigeon.initializeAuton(this);
            autoAim = false;
            if(robotState.isShooterReady()) {
                autoState = AutoState.test;
                timer.reset();
                timer.start();
            }
        }
        if (autoState == AutoState.test) {
            autonInProgress = true;
            driveRequested = false;
            hoodPosition = Shot.AUTO2;
            shoot = true;
            autoAim = false;
            
            if(timer.get() > 1) { // .5
                timer.reset();
                timer.start();
                drivetrain.initializeAuton(this);
                autoState = AutoState.getWallBall;
            }
        }
         // desiredState = new State(timer.get(), .6/1*timer.get(), .6/1, new Pose2d(lastDesiredState.poseMeters.getX()/2*timer.get(),
            //                                                         lastDesiredState.poseMeters.getY()-1.1/2*timer.get(),
            //                                                         lastDesiredState.poseMeters.getRotation()), 
            //                                                         1000);
        if (autoState == AutoState.getWallBall) {
            autonInProgress = true;
            driveRequested = true;
            deployRightIntake = true;
            deployLeftIntake = false;
            hoodPosition = Shot.TARMACK;
            autoAim = false;
            

                desiredState = new State(timer.get(), .6/1*timer.get(), .6/1, new Pose2d(lastDesiredState.poseMeters.getX()+.95/2*timer.get(),
                lastDesiredState.poseMeters.getY()-1.8/2*timer.get(),
                lastDesiredState.poseMeters.getRotation()), 
                1000);

            setTargetTheta(Rotation2d.fromDegrees(0));
            
            SmartDashboard.putNumber("TargetX", desiredState.poseMeters.getTranslation().getX());
            SmartDashboard.putNumber("TargetY", desiredState.poseMeters.getTranslation().getY());
            SmartDashboard.putNumber("TargetTheta", desiredState.poseMeters.getRotation().getDegrees());

            if (timer.get() > 2.2) {
                lastDesiredState = desiredState;
                autoState = AutoState.driveToSecondBall_1;
                timer.reset();
                timer.start();
            }                  
        }

        if (autoState == AutoState.driveToSecondBall_1) {
            autonInProgress = true;
            driveRequested = true;
            deployRightIntake = false;
            deployLeftIntake = false;
            hoodPosition = Shot.TARMACK;
            autoAim = false;

            desiredState = new State(timer.get(), .1/1*timer.get(), .1/1, new Pose2d(lastDesiredState.poseMeters.getX()-1/1*timer.get(),
                                                                    lastDesiredState.poseMeters.getY()+.08/1*timer.get(),
                                                                    lastDesiredState.poseMeters.getRotation()), 
                                                                    1000);
            setTargetTheta(Rotation2d.fromDegrees(0));

            SmartDashboard.putNumber("TargetX", desiredState.poseMeters.getTranslation().getX());
            SmartDashboard.putNumber("TargetY", desiredState.poseMeters.getTranslation().getY());
            SmartDashboard.putNumber("TargetTheta", desiredState.poseMeters.getRotation().getDegrees());
            if (timer.get() > 1) {
                autoState = AutoState.driveToSecondBall_2;
                lastDesiredState = desiredState;
                timer.reset();
                timer.start();
            }
        }
        if(autoState == AutoState.driveToSecondBall_2){
            autonInProgress = true;
            driveRequested = true;
            deployRightIntake = false;
            deployLeftIntake = true;
            autoAim = false;
            hoodPosition = Shot.TARMACK;

            desiredState = new State(timer.get(), .075/1.25*timer.get(),  .075/1.5, new Pose2d(lastDesiredState.poseMeters.getX()-1.4/1.25*timer.get(),
                                                                    lastDesiredState.poseMeters.getY()+1.4/1.25*timer.get(),
                                                                    lastDesiredState.poseMeters.getRotation()), 
                                                                    1000);
            setTargetTheta(Rotation2d.fromDegrees(60));

            SmartDashboard.putNumber("TargetX", desiredState.poseMeters.getTranslation().getX());
            SmartDashboard.putNumber("TargetY", desiredState.poseMeters.getTranslation().getY());
            SmartDashboard.putNumber("TargetTheta", desiredState.poseMeters.getRotation().getDegrees());
            if (timer.get() > 1.25) {
                autoState = AutoState.firstShot;
                lastDesiredState = desiredState;
                timer.reset();
                timer.start();
            }
        }
        if (autoState == AutoState.firstShot) {
            autonInProgress = true;
            driveRequested = false;
            deployRightIntake = false;
            deployLeftIntake = true;
            hoodPosition = Shot.TARMACK;
            desiredState = lastDesiredState;
            // setTargetTheta(Rotation2d.fromDegrees(60));
            autoAim = false;
            shoot = true;
            
            SmartDashboard.putNumber("TargetX", desiredState.poseMeters.getTranslation().getX());
            SmartDashboard.putNumber("TargetY", desiredState.poseMeters.getTranslation().getY());
            SmartDashboard.putNumber("TargetTheta", desiredState.poseMeters.getRotation().getDegrees());

            if (timer.get() > 1) {
                autoState = AutoState.driveToHumanPlayer;
                lastDesiredState = desiredState;
                timer.reset();
                timer.start();
            }                  
        }
        if (autoState == AutoState.driveToHumanPlayer) {
            autonInProgress = true;
            driveRequested = true;
            deployRightIntake = false;
            deployLeftIntake = false;
            hoodPosition = Shot.TARMACK;
            autoAim = false;

            desiredState = new State(timer.get(), .075/1.5*timer.get(),  .075/1.5, new Pose2d(lastDesiredState.poseMeters.getX()-4.2/1.5*timer.get(),
                                                                    lastDesiredState.poseMeters.getY()-.64/1.5*timer.get(),
                                                                    lastDesiredState.poseMeters.getRotation()), 
                                                                    1000);
            setTargetTheta(Rotation2d.fromDegrees(90));

            SmartDashboard.putNumber("TargetX", desiredState.poseMeters.getTranslation().getX());
            SmartDashboard.putNumber("TargetY", desiredState.poseMeters.getTranslation().getY());
            SmartDashboard.putNumber("TargetTheta", desiredState.poseMeters.getRotation().getDegrees());
            if (timer.get() > 1.5) {
                autoState = AutoState.pickUpHumanBalls;
                lastDesiredState = desiredState;
                timer.reset();
                timer.start();
            }              
        }
        if(autoState == AutoState.pickUpHumanBalls){
            hoodPosition = Shot.PROTECTED;
            if (timer.get() > 1.5) {
                autoState = AutoState.goToLastShot;
                lastDesiredState = desiredState;
                timer.reset();
                timer.start();
            }
        }
        if(autoState == AutoState.goToLastShot){
            autonInProgress = true;
            driveRequested = true;
            deployRightIntake = false;
            deployLeftIntake = false;
            hoodPosition = Shot.PROTECTED;
            autoAim = false;

            desiredState = new State(timer.get(), .075/1.5*timer.get(),  .075/1.5, new Pose2d(lastDesiredState.poseMeters.getX()+1.67/1.5*timer.get(),
                                                                    lastDesiredState.poseMeters.getY()/1.5*timer.get(),
                                                                    lastDesiredState.poseMeters.getRotation()), 
                                                                    1000);
            setTargetTheta(Rotation2d.fromDegrees(42));

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
            hoodPosition = Shot.TARMACK;
            autoAim = false;
            shoot = true;
            
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




    @Override
    public boolean getAutoIntakeDeploy() {
        // TODO Auto-generated method stub
        return false;
    }
}
