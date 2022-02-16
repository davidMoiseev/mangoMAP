package frc.robot;

import frc.robot.Autons.AutoRunner;

public class AutonCommader extends RobotCommander{

    AutoRunner auton;

    public AutonCommader(RobotState robotState) {
        auton = new AutoRunner(robotState);
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
}
