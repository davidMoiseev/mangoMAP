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

    @Override
    public boolean getClimberChangeState() {
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
}
