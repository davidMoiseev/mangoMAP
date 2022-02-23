package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import org.hotutilites.hotlogger.HotLogger;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.sensors.Pigeon;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.BallSupervisor;
import frc.robot.subsystems.Climber;
import static frc.robot.Constants.*;

public class Robot extends TimedRobot {

  private RobotState robotState;
  private TeleopCommander teleopCommander;
  private AutonCommader autonCommander;
  private Drivetrain drivetrain;
  private Pigeon pigeon;
  private PneumaticHub hub;
  private BallSupervisor ballSupervisor;
  private Climber climber;


  @Override
  public void robotInit() {
    HotLogger.Setup("Theta", "Left Front Absolute", "Left Front Assumed", 
                "Right Front Absolute", "Right Front Assumed",
                "Left Rear Absolute", "Left Rear Assumed",
                "Right Rear Absolute", "Right Rear Assumed", "ClimberCmd", 
                "LeftIntakeCmd", "RightIntakeCmd", "LeftShooterSpeed", "RightShooterSpeed");

    robotState = new RobotState();
    hub = new PneumaticHub(PNEUMATIC_HUB);
    teleopCommander = new TeleopCommander(robotState);
    autonCommander = new AutonCommader(robotState);
    pigeon = new Pigeon(robotState);
    drivetrain = new Drivetrain(robotState);
    ballSupervisor = new BallSupervisor(robotState, hub);
    climber = new Climber(robotState, hub);
  }

  @Override
  public void robotPeriodic() {
    pigeon.updateState(robotState, teleopCommander);
    drivetrain.updateState();
    drivetrain.logData();
  }

  @Override
  public void disabledInit() {
    drivetrain.zeroActuators();
    hub.disableCompressor();
  }

  @Override
  public void disabledPeriodic() {
    drivetrain.disabledAction(robotState);
  }

  @Override
  public void autonomousInit() {
    hub.enableCompressorDigital();
  }

  @Override
  public void autonomousPeriodic() {
    drivetrain.enabledAction(robotState, autonCommander);
    ballSupervisor.enabledAction(robotState, autonCommander);
    climber.enabledAction(robotState, autonCommander);
  }

  @Override
  public void teleopInit() {
    pigeon.zeroSensor();
    drivetrain.zeroActuators();
    drivetrain.zeroSensor();
    hub.enableCompressorDigital();
  }

  @Override
  public void teleopPeriodic() {
    drivetrain.enabledAction(robotState, teleopCommander);
    ballSupervisor.enabledAction(robotState, teleopCommander);
    climber.enabledAction(robotState, teleopCommander);
  }
}
