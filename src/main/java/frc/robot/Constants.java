// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class Constants {
    public static boolean realBot = true;

    public static final double DRIVETRAIN_TRACKWIDTH_METERS = realBot ? .5842 : 0.4445;

    public static final double DRIVETRAIN_WHEELBASE_METERS = realBot ? .5842 : 0.4445;

    public static final double MAX_VOLTAGE = 12.0;

    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
        SdsModuleConfigurations.MK4I_L2.getDriveReduction() *
        SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;

    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
        Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

    public static final int DRIVETRAIN_PIGEON_ID = realBot ? 20 : 50;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = realBot ? 1 : 1;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = realBot ? 2 : 3;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = realBot ? 16 : 13;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = realBot ? -Math.toRadians(132.363) : -Math.toRadians(21.97265625);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = realBot ? 3 : 5;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = realBot ? 4 : 7;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = realBot ? 17 : 17;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = realBot ? -Math.toRadians(180.204) : -Math.toRadians(58.71093750000001);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = realBot ? 5 : 2;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = realBot ? 6 : 4;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = realBot ? 18 : 14;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = realBot ? -Math.toRadians(33.486) : -Math.toRadians(309.27886962890625);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = realBot ? 7 : 6;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = realBot ? 8 : 8;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = realBot ? 19 : 18;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = realBot ? -Math.toRadians(11.689) : -Math.toRadians(212.2890625);

    public static final int RIGHT_INTAKE_MOTOR = 10;
    public static final int LEFT_INTAKE_MOTOR = 9;
}