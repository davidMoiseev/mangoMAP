package frc.robot;

import javax.swing.text.AbstractDocument.AbstractElement;

public abstract class RobotCommander {

    public abstract double getForwardCommand();
    public abstract double getStrafeCommand();
    public abstract double getTurnCommand();
    public abstract boolean getRunLeftIntake();
    public abstract boolean getRunRightIntake();
}
