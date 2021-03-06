package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.Constants.*;

public abstract class AutonCommader extends RobotCommander{

    protected RobotState robotState;
    protected boolean resetIMU = false;
    protected double resetIMUAngle = 0.0;
    private Timer timer;
    protected State desiredState;
    private Rotation2d targetTheta;
    
    public abstract String getName();

    public Rotation2d getTargetTheta() {
        return targetTheta;
    }

    public void setTargetTheta(Rotation2d targetTheta) {
        this.targetTheta = targetTheta;
    }

    public AutonCommader(RobotState robotState) {
        this.robotState = robotState;
        setTargetTheta(new Rotation2d());
        
        timer = new Timer();
    }

    public AutonCommader(RobotState robotState, String trajectoryJSON) {
        this.robotState = robotState;
        setTargetTheta(new Rotation2d());
 
        timer = new Timer();
    }

    public abstract Pose2d getInitialPose();

    @Override
    public boolean getResetIMU() {
        return resetIMU;
    }
    
    public double getResetIMUAngle() {
        return resetIMUAngle;
    }

    @Override
    public double getClimberMotor() {
        // TODO Auto-generated method stub
        return 0;
    }
    
    public State getDesiredState() {
        return desiredState;
    }

    public abstract void initializeAuton();

    public abstract void updateCommand();

    public abstract boolean getAutonInProgress();
    public abstract boolean getDriveRequested();

    protected Trajectory readTrajectoryFile(String fileName) {
        Trajectory trajectory;
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(fileName);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
         } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + fileName, ex.getStackTrace());
            trajectory = null;
         }
         return trajectory;
    }

    @Override
    public int getShooterSpeedThreshHold() {
      return AUTO_SHOOTER_OK_SPEED_TOLERANCE;
    }

    @Override
    public double getShooterTimeThreshHold() {
      return AUTO_SHOOTER_OK_TIME_OVER_SPEED_TOLERANCE;
    }

    public boolean[] getBallivator(){
        boolean[] x = {false, false};
        return x;
    }

    @Override
    public boolean getClimberExtend() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean getClimberRetract() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean getClimberManualControl() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean getAbuttonHeld() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean getBbuttonHeld() {
        // TODO Auto-generated method stub
        return false;
    }
}
