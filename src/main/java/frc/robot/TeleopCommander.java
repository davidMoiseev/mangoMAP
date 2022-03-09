package frc.robot;

import edu.wpi.first.wpilibj.Controller;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.Shot;

import static frc.robot.Constants.*;

public class TeleopCommander extends RobotCommander{

    private static XboxController driver;
    private static XboxController operator;
    private static boolean climberUp;
    Shot hoodPosition = Shot.NEUTRAL;
    double shooterSpeed;
    boolean shooterOn;
    boolean autoAimMode = false;
    boolean climbState = false;
    boolean downCalled = false;
    boolean upCalled = false;
    double climberAngle = (PACKAGE_ANGLE);

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
    public double getClimberMotor() {
      if (robotState.getClimberExtended() == true) {
        return deadband(operator.getLeftY(), 0.25, 0.9);
      } else {
        return 0.0;
      }
    }

    @Override
    public boolean getClimberRelease() {
      if (operator.getXButton() && robotState.getClimberExtended() == true) {
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
      public Shot getHoodPosition() {
        if (robotState.getClimberExtended() == true) {
          this.hoodPosition = Shooter.Shot.CLIMB;
          this.shooterOn = false;
        } else if (operator.getAButtonPressed()) {
          this.hoodPosition = Shooter.Shot.FENDER;
          this.shooterOn = true;
        } else if (operator.getXButtonPressed()) {
          this.hoodPosition=  Shooter.Shot.WALL;
          this.shooterOn = true;
        } else if (operator.getBButtonPressed()) {
          this.hoodPosition = Shooter.Shot.TARMACK;
          this.shooterOn = true;
        } else if (operator.getYButtonPressed()) {
          this.hoodPosition = Shooter.Shot.PROTECTED;
          this.shooterOn = true;
        } else if (operator.getBackButtonPressed()){
          hoodPosition = Shot.NEUTRAL;
        }
        return this.hoodPosition;
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

      @Override
      public boolean getOverrideShooterMotor() {
        return operator.getBackButton();
      }

      @Override
      public boolean getOverrideBallivatorMotor() {
        return operator.getBackButton();
      }

      @Override
      public boolean getOverrideIntakmotor() {
        return operator.getBackButton();
      }

      @Override
      public int getShooterSpeedThreshHold() {
        return TELE_SHOOTER_OK_SPEED_TOLERANCE;
      }

      @Override
      public double getShooterTimeThreshHold() {
        return TELE_SHOOTER_OK_TIME_OVER_SPEED_TOLERANCE;
      }

      @Override
      public boolean getClimberExtend() {
        if ((robotState.getClimberExtended() == false) && ((operator.getPOV() < 20 || operator.getPOV() > 340) && (operator.getPOV() != -1))) {
          return true;
        } else {
          return false;
        }
      }

      @Override
      public boolean getClimberRetract() {
        if ((robotState.getClimberExtended() == true) && (operator.getPOV() > 160 && operator.getPOV() < 200)) {
          return true;
        } else {
          return false;
        }
      }

      @Override
      public boolean getClimberManualControl() {
        if ((robotState.getClimberExtended() == true) && (Math.abs(operator.getLeftY()) > 0.2)) {
          return true;
        } else {
          return false;
        }
      }

      @Override
      public boolean getAbuttonHeld() {
        return operator.getAButton();
      }

      @Override
      public boolean getBbuttonHeld() {
        return operator.getBButton();
      }
}
