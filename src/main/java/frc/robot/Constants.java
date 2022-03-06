// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class Constants {
    public static boolean realBot = true;

    public static final double DRIVETRAIN_TRACKWIDTH_METERS = realBot ? 0.57785 : 0.4445;

    public static final double DRIVETRAIN_WHEELBASE_METERS = realBot ? 0.57785 : 0.4445;

    public static final double MAX_VOLTAGE = 12.0;

    public static final double MAX_VELOCITY_METERS_PER_SECOND = realBot ?
        6380.0 / 60.0 *
        SdsModuleConfigurations.MK4I_L2.getDriveReduction() *
        SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI
        :
        6380.0 / 60.0 *
        SdsModuleConfigurations.MK4_L2.getDriveReduction() *
        SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;

    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
        Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0));
    
    public static final int DRIVETRAIN_PIGEON_ID = realBot ? 20 : 50;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = realBot ? 1 : 1;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = realBot ? 2 : 3;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = realBot ? 16 : 13;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = realBot ? -Math.toRadians(219.55078125) : -Math.toRadians(21.97265625);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = realBot ? 3 : 5;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = realBot ? 4 : 7;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = realBot ? 17 : 17;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = realBot ? -Math.toRadians(181.93359375) : -Math.toRadians(58.71093750000001);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = realBot ? 5 : 2;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = realBot ? 6 : 4;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = realBot ? 18 : 14;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = realBot ? -Math.toRadians(325.634765625) : -Math.toRadians(309.27886962890625);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = realBot ? 7 : 6;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = realBot ? 8 : 8;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = realBot ? 19 : 18;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = realBot ? -Math.toRadians(348.92578125) : -Math.toRadians(212.2890625);

    public static final int RIGHT_INTAKE_MOTOR = 10;
    public static final int LEFT_INTAKE_MOTOR = 9;
    public static final double SHOOTER_SPEED_FENDER = 1100.0; //2200
    public static final double SHOOTER_SPEED_WALL = 2525.0;
    public static final double SHOOTER_SPEED_TARMACK = 2100.0;
    public static final double SHOOTER_SPEED_AUTO = 1930.0;
    public static final double SHOOTER_SPEED_PROTECTED = 2800.0;
    public static final double SHOOTER_GAIN_P = 0.1; // 0.0005
    public static final double SHOOTER_GAIN_I = 0.0003;  // 0.00005
    public static final double SHOOTER_GAIN_D = 0.0003; // 0.000012
    public static final double SHOOTER_FF = 0.05289;

    public static final int RIGHT_SHOOTER_MOTOR = 14;
    public static final int LEFT_SHOOTER_MOTOR = 15;

    public static final int RIGHT_BALLIVATOR_MOTOR = 11;
    public static final int LEFT_BALLIVATOR_MOTOR = 12;
    public static final int BALLIVATOR_SENSOR = 0;
    public static final int BALLIVATOR_SOLENOID = 0;
    public static final int TELE_SHOOTER_OK_SPEED_TOLERANCE = 9000;
    public static final double TELE_SHOOTER_OK_TIME_OVER_SPEED_TOLERANCE = 0;
    public static final int AUTO_SHOOTER_OK_SPEED_TOLERANCE = 100;
    public static final double AUTO_SHOOTER_OK_TIME_OVER_SPEED_TOLERANCE = .1;

    public static final int PNEUMATIC_HUB = 23;
    public static final int LEFT_INTAKE_FWD_SOLENOID = 13;
    public static final int LEFT_INTAKE_REV_SOLENOID = 2;
    public static final int RIGHT_INTAKE_FWD_SOLENOID = 12;
    public static final int RIGHT_INTAKE_REV_SOLENOID = 3;
    public static final int CLIMBER_EXTEND_FWD_SOLENOID = 9;
    public static final int CLIMBER_EXTEND_REV_SOLENOID = 6;
    public static final int CLIMBER_RELEASE_SOLENOID = 8;
    public static final double MINIMUM_PRESSURE = 90.0;
    public static final double MAXIMUM_PRESSURE = 110.0;

    public static final int SHOOTER_INSIDE_SOLENOID_FOWARD = 11;
    public static final int SHOOTER_INSIDE_SOLENOID_REVERSE = 4;
    public static final int SHOOTER_OUSIDE_SOLENOID_FOWARD = 10;
    public static final int SHOOTER_OUTSIDE_SOLENOID_REVERSE = 5;

    public static final int CLIMBER_MOTOR = 13;


    //AutoAim
    public static final double ALLOWED_OFFSET = 5;
    public static final double ALLOWED_Y_OFFSET = 4;
    public static final double ALLOWED_X_OFFSET = 4;

    public static final double X_ADJUST_SPEED = 0.2;
    public static final double Y_ADJUST_SPEED = 0.04;
    public static final double STRAFE_SPEED = 0.5;

    public static final double Y_OFFSET = 0.06;
}