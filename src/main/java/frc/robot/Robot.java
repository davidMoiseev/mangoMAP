package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.fasterxml.jackson.databind.util.RootNameLookup;

import org.hotutilites.hotlogger.HotLogger;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.net.PortForwarder;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Autons.AutonLeft;
import frc.robot.Autons.AutonLeft4Ball;
import frc.robot.Autons.AutonLeftPlusBlue;
import frc.robot.Autons.AutonPost2Ball;
import frc.robot.Autons.AutonRight5Ball;
import frc.robot.Autons.AutonRight5Ballold;
import frc.robot.Autons.AutonRightBlue;
import frc.robot.Autons.AutonRightRed;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.Lights;
import edu.wpi.first.wpilibj.Timer;
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
  private double autonSelection;
  private Object autonSelectionPrev;
  private Lights lights;
  private AutonPost2Ball post2Ball;

  public Timer timer;

  @Override
  public void robotInit() {
    HotLogger.Setup("Theta", "Left Front Absolute", "Left Front Assumed",
        "Right Front Absolute", "Right Front Assumed",
        "Left Rear Absolute", "Left Rear Assumed",
        "Right Rear Absolute", "Right Rear Assumed", "ClimberCmd",
        "LeftIntakeCmd", "RightIntakeCmd", "LeftShooterSpeed", "RightShooterSpeed", "targetRPM",
        "climberState", "actualPosTicks", "actualPosDeg", "targetPosDeg", "targetPosTicks",
        "X Pigeon", "Y Pigeon", "Z Pigeon", "Pitch", "Roll", "ClimberReleased",
        "shooterError", "hoodPosition", "TargetX", "TargetY", "TargetTheta", "Robot State Theta", "poseX", "poseY", "Compressor Cur", "ClimberSpeed", "ClimberCurrent",
        "BallivatorTop", "BallivatorBottom", "NumBalls",
        "Current Draw 1", "Current Draw 3", "Current Draw 5", "Current Draw 7",
        "Current Supply 1", "Current Supply 3", "Current Supply 5", "Current Supply 7");


    robotState = new RobotState();
    hub = new PneumaticHub(PNEUMATIC_HUB);
    teleopCommander = new TeleopCommander(robotState);
    pigeon = new Pigeon(robotState);
    drivetrain = new Drivetrain(robotState);
    ballSupervisor = new BallSupervisor(robotState, hub, drivetrain);
    climber = new Climber(robotState, hub);
    limelight = new Limelight(robotState);
    lights = new Lights(robotState);

    timer = new Timer();
    
    post2Ball = new AutonPost2Ball(robotState);

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);

    // PortForwarder.add(5800, "limelight.local", 5800);
    // PortForwarder.add(5801, "limelight.local", 5801);
    // PortForwarder.add(5801, "limelight.local", 5802);
    // PortForwarder.add(5801, "limelight.local", 5803);
    // PortForwarder.add(5801, "limelight.local", 5804);
    // PortForwarder.add(5805, "limelight.local", 5805);


    autonSelection = 0; // RIGHT_RED, RIGHT_BLUE, LEFT, RIGHT_5
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

    HotLogger.Log("Compressor Cur", hub.getCompressorCurrent());
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
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);

    climber.disabledAction(robotState);
    
    lights.setLightsDisable();
    drivetrain.disabledAction(robotState);
    // SmartDashboard.getString("AutonSelection(LeftOrRight)", autonSelection);
    
    drivetrain.zeroActuators();
  }

  @Override
  public void autonomousInit() {
    autonSelection = SmartDashboard.getNumber("autonSelection", 0);

    // RIGHT_RED, RIGHT_BLUE, LEFT
    if (autonSelection == 0) {
      selectedAuton = new AutonRightRed(robotState);
      // SmartDashboard.putString("AutonSelected", selectedAuton.getName());
    } else if (autonSelection == 1) {
      selectedAuton = new AutonRightBlue(robotState);
      // SmartDashboard.putString("AutonSelected", selectedAuton.getName());
    } else if (autonSelection == 2) {
      // selectedAuton = new AutonLeft(robotState);
      selectedAuton = new AutonLeftPlusBlue(robotState);
    } else if (autonSelection == 3){
      selectedAuton = new AutonRight5Ball(robotState);
    } else if (autonSelection == 4){
      selectedAuton = new AutonLeft4Ball(robotState);
    } else {
      // SmartDashboard.putString("AutonSelected", "ERROR no autonomous file selected ERROR");
      selectedAuton = new AutonRightBlue(robotState);
    }

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
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);

    if (autonSelection == 0) {
      ((AutonRightRed)selectedAuton).updateCommand(pigeon, drivetrain);
    } else if (autonSelection == 1) {
      ((AutonRightBlue)selectedAuton).updateCommand(pigeon, drivetrain);
    } else if (autonSelection == 2) {
      // ((AutonLeft)selectedAuton).updateCommand(pigeon, drivetrain);
      ((AutonLeftPlusBlue)selectedAuton).updateCommand(pigeon, drivetrain);
    } else if (autonSelection == 3){
      ((AutonRight5Ball)selectedAuton).updateCommand(pigeon, drivetrain);
    } else if (autonSelection == 4){
      ((AutonLeft4Ball)selectedAuton).updateCommand(pigeon, drivetrain);
    } else {
      ((AutonRightBlue)selectedAuton).updateCommand(pigeon, drivetrain);
    }
    drivetrain.autonenabledAction(selectedAuton);
    ballSupervisor.enabledAction(robotState, selectedAuton);
    lights.setLightsAuton();
  }

  @Override
  public void teleopInit() {
    timer.reset();
    timer.start();
    //pigeon.zeroSensor();
    ballSupervisor.zeroSensor();
    drivetrain.zeroActuators();
    drivetrain.zeroSensor();
    hub.enableCompressorAnalog(MINIMUM_PRESSURE, MAXIMUM_PRESSURE);
    lights.setLightsTeleop(); 
  }

  @Override
  public void teleopPeriodic() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);

    pigeon.enabledAction(teleopCommander);
    drivetrain.enabledAction(robotState, teleopCommander);
    ballSupervisor.enabledAction(robotState, teleopCommander);
    climber.enabledAction(robotState, teleopCommander);
    SmartDashboard.putBoolean("Button A", driver.getAButton());
    lights.setLightsTeleop();
  }

}
