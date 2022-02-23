package frc.robot;

import javax.swing.text.AbstractDocument.AbstractElement;

public abstract class RobotCommander {

    public abstract double getForwardCommand();
    public abstract double getStrafeCommand();
    public abstract double getTurnCommand();
    public abstract boolean getRunLeftIntake();
    public abstract boolean getRunRightIntake();
    public abstract boolean getClimberExtend();
    public abstract double getClimberMotor();
    public abstract boolean getClimberRelease();
    public abstract double getRightIntakeCommand();
    public abstract double getLeftIntakeCommand();
    public abstract boolean getRobotAim();
}
