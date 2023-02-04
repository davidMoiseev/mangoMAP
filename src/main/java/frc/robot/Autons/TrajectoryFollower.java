package frc.robot.Autons;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

public class TrajectoryFollower{
    private Timer timer = new Timer();
    private Trajectory trajectory;
    private Pose2d pose;
    private SwerveDriveKinematics kinematics;
    private HolonomicDriveController controller;
    private SwerveModuleState[] outputStates;
    private Rotation2d desiredRotation;

    public TrajectoryFollower(
        Trajectory m_trajectory,
        Pose2d m_pose,
        SwerveDriveKinematics m_kinematics,
        PIDController xController,
        PIDController yController,
        ProfiledPIDCommand thetaController
    ){
        
    }
}