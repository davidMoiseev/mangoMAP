package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

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
        return modifyAxis(driver.getLeftY());
    }
    public double getStrafeCommand(){
        return modifyAxis(driver.getLeftX());
    }
    public double getTurnCommand(){
        return deadband(driver.getRightX(), .3);
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
