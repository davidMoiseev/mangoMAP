package frc.robot;

import javax.swing.text.AbstractDocument.AbstractElement;

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
    public abstract boolean getRobotAim();
    public abstract int getHoodPosition();
    public abstract double getShooterSpeed();
    public abstract boolean[] getBallivator();
    public abstract boolean getClimberExtend();
    public abstract boolean getClimberRetract();
    public abstract boolean getClimberManualControl();
    public abstract boolean getAbuttonHeld();
    public abstract boolean getBbuttonHeld();
}
