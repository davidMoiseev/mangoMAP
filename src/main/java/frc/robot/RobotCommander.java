package frc.robot;

import javax.swing.text.AbstractDocument.AbstractElement;

import frc.robot.subsystems.Shooter.Shot;

public abstract class RobotCommander {

    public abstract double getForwardCommand();
    public abstract double getStrafeCommand();
    public abstract double getTurnCommand();
    public abstract boolean getRunLeftIntake();
    public abstract boolean getRunRightIntake();
    public abstract double getClimberMotor();
    public abstract boolean getClimberRelease();
    public abstract double getRightIntakeCommand();
    public abstract double getLeftIntakeCommand();
    public abstract boolean getResetIMU();
    public abstract double getResetIMUAngle();
    public abstract boolean getRobotAim();
    public abstract Shot getHoodPosition();
    public abstract boolean[] getBallivator();
    public abstract boolean getOverrideShooterMotor();
    public abstract boolean getOverrideBallivatorMotor();
    public abstract boolean getOverrideIntakmotor();
    public abstract int getShooterSpeedThreshHold();
    public abstract double getShooterTimeThreshHold();
    public abstract boolean getClimberExtend();
    public abstract boolean getClimberRetract();
    public abstract boolean getClimberManualControl();
    public abstract boolean getAbuttonHeld();
    public abstract boolean getBbuttonHeld();
}
