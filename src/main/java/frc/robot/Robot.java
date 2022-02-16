package frc.robot;

import org.hotutilites.hotlogger.HotLogger;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.sensors.Pigeon;
import frc.robot.subsystems.Drivetrain;

public class Robot extends TimedRobot {

  private RobotState robotState;
  private TeleopCommander teleopCommander;
  private AutonCommader autonCommader;
  private Drivetrain drivetrain;
  private Pigeon pigeon;

  @Override
  public void robotInit() {
    HotLogger.Setup("Theta");

    robotState = new RobotState();

    teleopCommander = new TeleopCommander(robotState);
    autonCommader = new AutonCommader(robotState);

    pigeon = new Pigeon(robotState);
    drivetrain = new Drivetrain(robotState);
  }

  @Override
  public void robotPeriodic() {
    pigeon.updateState();
    drivetrain.updateState();
  }

  @Override
  public void disabledInit() {
    drivetrain.zeroActuators();
  }

  @Override
  public void disabledPeriodic() {
    drivetrain.disabledAction(robotState);
  }

  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {
    drivetrain.enabledAction(robotState, autonCommader);

  }

  @Override
  public void teleopInit() {
    pigeon.zeroSensor();
    drivetrain.zeroActuators();
    drivetrain.zeroSensor();
  }

  @Override
  public void teleopPeriodic() {
    drivetrain.enabledAction(robotState, teleopCommander);
  }
}
