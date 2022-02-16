package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

import static frc.robot.Constants.*;

public class TeleopCommander extends RobotCommander{

    private static XboxController driver;
    private static XboxController operator;

    RobotState robotState;

    public TeleopCommander(RobotState robotState) {
        this.robotState = robotState;
        driver = new XboxController(0);
        operator = new XboxController(1);
    }

    public double getForwardCommand(){
        return modifyAxis(driver.getLeftY()) * MAX_VELOCITY_METERS_PER_SECOND;
    }
    public double getStrafeCommand(){
        return modifyAxis(driver.getLeftX()) * MAX_VELOCITY_METERS_PER_SECOND;
    }
    public double getTurnCommand(){
        return -deadband(driver.getRightX(), .3) * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    }

    public boolean getRunRightIntake(){
      return driver.getRightBumper();
    }

    public boolean getRunLeftIntake(){
      return driver.getLeftBumper();
    }


    private static double deadband(double value, double deadband){
        if(Math.abs(value) < deadband){
          return 0;
        } else {
          return value * .4;
        }
      }
    
      private static double modifyAxis(double value) {
        boolean deadband = 0.3 > Math.sqrt(Math.pow(driver.getLeftX(), 2) + Math.pow(driver.getLeftY(), 2));
    
        if (deadband) {
          return 0;
        } else {
          return value * 0.33;
        }
    }
}
