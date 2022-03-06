package frc.robot;

import edu.wpi.first.wpilibj.Controller;
import edu.wpi.first.wpilibj.XboxController;

import static frc.robot.Constants.*;

public class TeleopCommander extends RobotCommander{

    private static XboxController driver;
    private static XboxController operator;
    private static boolean climberUp;
    int hoodPosition = 1;
    double shooterSpeed;
    boolean shooterOn;
    boolean autoAimMode = false;
    boolean climbState = false;
    boolean downCalled = false;
    boolean upCalled = false;

    RobotState robotState;

    public TeleopCommander(RobotState robotState) {
        this.robotState = robotState;
        driver = new XboxController(0);
        operator = new XboxController(1);
        climberUp = false;
        shooterSpeed = 0.0;
        shooterOn = false;
    }

    public double getForwardCommand(){
        return -modifyAxis(driver.getLeftY()) * MAX_VELOCITY_METERS_PER_SECOND;
    }
    public double getStrafeCommand(){
        return -modifyAxis(driver.getLeftX()) * MAX_VELOCITY_METERS_PER_SECOND;
    }
    public double getTurnCommand(){
        double value = deadband(driver.getRightX(), 0.3, 0.4) * (MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND) * 0.5;

        return - (Math.abs(value) * value);
    }

    public boolean getRunRightIntake(){
      return operator.getRightBumper();
    }

    public boolean getRunLeftIntake(){
      return operator.getLeftBumper();
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
        return -deadband(temp, 0.2, 1.0);
      } else {
        return 0.0;
      }
    }

    @Override
    public boolean getClimberChangeState() {
      if ((operator.getPOV() < 20 || operator.getPOV() > 340) && (operator.getPOV() != -1)) {
        if (downCalled == false){
          downCalled = true;
          upCalled = false;
          return true;
        }
      } else if (operator.getPOV() > 160 && operator.getPOV() < 200) {
        if (upCalled == false){
          downCalled = false;
          upCalled = true;
          return true;
        }
      }
      return false;
    }

    @Override
    public double getClimberMotor() {
      return deadband(operator.getLeftY(), 0.2, 0.9);
    }

    @Override
    public boolean getClimberRelease() {
      if (operator.getPOV() > 70 && operator.getPOV() < 110) {
        return true;
      }
      else{
        return false;
      }
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
          return Math.abs(value) * value;
        }
    }

      @Override
      public boolean getRobotAim() {
        return driver.getAButton();
      }

      @Override
      public boolean getResetIMU() {
        // TODO Auto-generated method stub
        return driver.getBackButton();
      }


      public double getResetIMUAngle() {
        return 0;
      }

      @Override
      public int getHoodPosition() {
        if (operator.getAButtonPressed()) {
          this.hoodPosition = 1;
          this.shooterOn = true;
        } else if (operator.getBButtonPressed()) {
          this.hoodPosition=  2;
          this.shooterOn = true;
        } else if (operator.getXButtonPressed()) {
          this.hoodPosition = 3;
          this.shooterOn = true;
        } else if (operator.getYButtonPressed()) {
          this.hoodPosition = 4;
          this.shooterOn = true;
        }
        return this.hoodPosition;
      }

      @Override
      public double getShooterSpeed() {
        if (operator.getBackButton() || shooterOn == false) {
          this.shooterSpeed = 0.0;
          this.shooterOn = false;
        }

        if (this.shooterOn) {
          if (hoodPosition == 1) {
            this.shooterSpeed = SHOOTER_SPEED_1;
          } else if (hoodPosition == 2) {
            this.shooterSpeed = SHOOTER_SPEED_2;
          } else if (hoodPosition == 3) {
            this.shooterSpeed = SHOOTER_SPEED_3;
          } else if (hoodPosition == 4) {
            this.shooterSpeed = SHOOTER_SPEED_4;
          }
        }
        return this.shooterSpeed;
      }

      public boolean[] getBallivator(){
        boolean RT= false, LT = false, dRT = false, enable = operator.getStartButton(), stop = operator.getBackButtonReleased();
        if(operator.getRightTriggerAxis() > .5){
          RT= true;
        } else {
          RT= false;
        }
        if(operator.getLeftTriggerAxis() > .5){
          LT = true;
        } else {
          LT = false;
        }
        if(driver.getRightTriggerAxis() > .5){
          dRT = true;
        } else {
          dRT = false;
        }
        boolean[] tmp = {RT, LT, enable, dRT, stop};
        return tmp;
      }
}
