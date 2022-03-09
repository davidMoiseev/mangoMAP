package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.fasterxml.jackson.databind.util.RootNameLookup;

import org.hotutilites.hotlogger.HotLogger;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Autons.AutonLeft;
import frc.robot.Autons.AutonRight;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.Lights;
import frc.robot.sensors.Pigeon;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.BallSupervisor;
import frc.robot.subsystems.Ballivator;
import frc.robot.subsystems.Climber;
import static frc.robot.Constants.*;

public class Robot extends TimedRobot {

  private RobotState robotState;
  private TeleopCommander teleopCommander;
  private Drivetrain drivetrain;
  private Pigeon pigeon;
  private PneumaticHub hub;
  private BallSupervisor ballSupervisor;
  private Climber climber;
  private Limelight limelight;
  XboxController driver = new XboxController(0);
  private AutonCommader selectedAuton;
  private String autonSelection;
  private Object autonSelectionPrev;
  private Lights lights;

  @Override
  public void robotInit() {
    HotLogger.Setup("Theta", "Left Front Absolute", "Left Front Assumed",
        "Right Front Absolute", "Right Front Assumed",
        "Left Rear Absolute", "Left Rear Assumed",
        "Right Rear Absolute", "Right Rear Assumed", "ClimberCmd",
        "LeftIntakeCmd", "RightIntakeCmd", "LeftShooterSpeed", "RightShooterSpeed", "TargetSpeed",
        "climberState", "actualPosTicks", "actualPosDeg", "targetPosDeg", "targetPosTicks");

    robotState = new RobotState();
    hub = new PneumaticHub(PNEUMATIC_HUB);
    teleopCommander = new TeleopCommander(robotState);
    pigeon = new Pigeon(robotState);
    drivetrain = new Drivetrain(robotState);
    ballSupervisor = new BallSupervisor(robotState, hub);
    climber = new Climber(robotState, hub);
    limelight = new Limelight(robotState);
    lights = new Lights(robotState);
    
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);

    autonSelection = "LEFT";
  }

  @Override
  public void robotPeriodic() {
    pigeon.updateState();
    pigeon.logData();
    limelight.updateState();
    drivetrain.updateState();
    ballSupervisor.updateState();
    climber.updateState();
    drivetrain.logData();
    ballSupervisor.logData();
    climber.logData();
    lights.logData();
  }

  @Override
  public void disabledInit() {
    lights.setLightsDisable();
    drivetrain.zeroActuators();
    hub.disableCompressor();
    // drivetrain.setBrakeMode(false);
  }

  @Override
  public void disabledPeriodic() {
    lights.setLightsDisable();
    drivetrain.disabledAction(robotState);
    SmartDashboard.getString("AutonSelection(LeftOrRight)", autonSelection);
    
    drivetrain.zeroActuators();

    if (! autonSelection.equals(autonSelectionPrev)) {
      if (autonSelection.equals("Left")) {
        selectedAuton = new AutonLeft(robotState);
        SmartDashboard.putString("AutonSelected", selectedAuton.getName());
      } else if (autonSelection.equals("Right")) {
        selectedAuton = new AutonRight(robotState);
        SmartDashboard.putString("AutonSelected", selectedAuton.getName());
      } else {
        SmartDashboard.putString("AutonSelected", "ERROR no autonomous file selected ERROR");
        selectedAuton = new AutonRight(robotState);
      }
    }
  }

  @Override
  public void autonomousInit() {
    hub.enableCompressorAnalog(MINIMUM_PRESSURE, MAXIMUM_PRESSURE);
    selectedAuton.initializeAuton();
    drivetrain.initializeAuton(selectedAuton);
    drivetrain.zeroActuators();
    ballSupervisor.zeroSensor();
    pigeon.initializeAuton(selectedAuton);
    lights.setLightsAuton();

    // drivetrain.setBrakeMode(true);
  }

  @Override
  public void autonomousPeriodic() {
    ((AutonRight)selectedAuton).updateCommand(pigeon, drivetrain);
    drivetrain.autonenabledAction(selectedAuton);
    ballSupervisor.enabledAction(robotState, selectedAuton);
    lights.setLightsAuton();
  }

  @Override
  public void teleopInit() {
    //pigeon.zeroSensor();
    ballSupervisor.zeroSensor();
    drivetrain.zeroActuators();
    drivetrain.zeroSensor();
    hub.enableCompressorAnalog(MINIMUM_PRESSURE, MAXIMUM_PRESSURE);
    lights.setLightsTeleop();
  }

  @Override
  public void teleopPeriodic() {
    pigeon.enabledAction(teleopCommander);
    drivetrain.enabledAction(robotState, teleopCommander);
    ballSupervisor.enabledAction(robotState, teleopCommander);
    climber.enabledAction(robotState, teleopCommander);
    SmartDashboard.putBoolean("Button A", driver.getAButton());
    lights.setLightsTeleop();
  }
}
