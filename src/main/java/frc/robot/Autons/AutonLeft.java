package frc.robot.Autons;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.AutonCommader;
import frc.robot.RobotState;
import frc.robot.subsystems.Shooter.Shot;

public class AutonLeft extends AutonCommader {
    
    public static final String name = "LEFT";
    private Trajectory trajectory;

    public AutonLeft(RobotState robotState) {
        super(robotState,AutonLeftConstants.trajectoryJSON_FileName);
    }

    @Override
    public Pose2d getInitialPose() {
        return trajectory.getInitialPose();
    }

    @Override
    public String getName() {
        return name;
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
    public Shot getHoodPosition() {
        // TODO Auto-generated method stub
        return Shot.NEUTRAL;
    }

    @Override
    public boolean[] getBallivator() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public void initializeAuton() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void updateCommand() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public boolean getAutonInProgress() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public Rotation2d getTargetTheta() {
        // TODO Auto-generated method stub
        return null;
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
        return false;
    }
    
    
}
