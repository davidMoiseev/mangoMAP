package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

import static frc.robot.Constants.*;

public class TeleopCommander extends RobotCommander{

    private static XboxController driver;
    private static XboxController operator;
    private static boolean climberUp;

    RobotState robotState;

    public TeleopCommander(RobotState robotState) {
        this.robotState = robotState;
        driver = new XboxController(0);
        operator = new XboxController(1);
        climberUp = false;
    }

    public double getForwardCommand(){
        return modifyAxis(driver.getLeftY()) * MAX_VELOCITY_METERS_PER_SECOND;
    }
    public double getStrafeCommand(){
        return modifyAxis(driver.getLeftX()) * MAX_VELOCITY_METERS_PER_SECOND;
    }
    public double getTurnCommand(){
        return -deadband(driver.getRightX(), 0.3, 0.4) * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    }

    public boolean getRunRightIntake(){
      return driver.getRightBumper();
    }

    public boolean getRunLeftIntake(){
      return driver.getLeftBumper();
    }

    public double getLeftIntakeCommand() {
      if (this.getRunLeftIntake()) {
        double temp = operator.getRightY();
        return deadband(temp, 0.2, 1.0);
      } else {
        return 0.0;
      }
    }

    public double getRightIntakeCommand() {
      if (this.getRunRightIntake()) {
        double temp = operator.getRightY();
        return deadband(temp, 0.2, 1.0);
      } else {
        return 0.0;
      }
    }

    @Override
    public boolean getClimberExtend() {
      if (operator.getBButtonReleased()) {
        climberUp = !climberUp;
      }
      return climberUp;
    }

    @Override
    public double getClimberMotor() {
      return deadband(operator.getLeftY(), 0.2, 0.9);
    }

    @Override
    public boolean getClimberRelease() {
      return operator.getXButton();
    }

    private static double deadband(double value, double deadband, double maxRange){
      /* Schang changed math so command is scaled from the deadband->max as 0->1 */
      /* The maxRange arguement is to scale the joystick range to a maximum value */
        if(Math.abs(value) < deadband){
          return 0;
        } else if (value < 0) {
          return  ((value + deadband)/(1.0 - deadband)) * maxRange;
        } else {
          return  ((value - deadband)/(1.0 - deadband)) * maxRange;
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

      @Override
      public boolean getRobotAim() {
        return false;
      }

}
