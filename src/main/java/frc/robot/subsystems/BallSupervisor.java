package frc.robot.subsystems;

import org.hotutilites.hotInterfaces.IHotSensedActuator;
import org.hotutilites.hotcontroller.HotController;
import org.hotutilites.hotlogger.HotLogger;

import edu.wpi.first.wpilibj.PneumaticHub;
import frc.robot.RobotCommander;
import frc.robot.RobotState;
import static frc.robot.Constants.*;

public class BallSupervisor extends SubsystemBase{
    RobotState robotState;
    
    Shooter shooter;
    Intake intake;
    Ballivator ballivator;

    public BallSupervisor(RobotState robotState, PneumaticHub hub){
        this.robotState = robotState;
        Intake intake = new Intake(robotState, hub);
        Ballivator ballivator = new Ballivator(robotState);
        Shooter shooter = new Shooter(robotState, hub);
    }
    
    @Override
    public void enabledAction(RobotState robotState, RobotCommander commander) {
        intake.enabledAction(robotState, commander);
        //ballivator.enabledAction(robotState, commander);
        shooter.enabledAction(robotState, commander);
    }

    @Override
    public void disabledAction(RobotState robotState) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void updateState() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void zeroActuators() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void zeroSensor() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void logData() {
        // TODO Auto-generated method stub
        
    }
    
}
