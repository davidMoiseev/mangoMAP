package frc.robot.subsystems;

import frc.robot.RobotCommander;
import frc.robot.RobotState;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.Constants.*;

public class Lights extends SubsystemBase{
    AddressableLED m_led;
    AddressableLEDBuffer m_LedBuffer;
    private RobotState robotState;
    
     public Lights(RobotState robotState){
       this.robotState = robotState;
       m_led = new AddressableLED(0);
       m_LedBuffer = new AddressableLEDBuffer(24);
       m_led.setLength(m_LedBuffer.getLength());
       for (int i = 0; i < m_LedBuffer.getLength(); i++) { 
           m_LedBuffer.setRGB(i, 0, 0, 0); 
        }
       m_led.setData(m_LedBuffer);
       m_led.start();
     }

    @Override
    public void enabledAction(RobotState robotState, RobotCommander commander) {
        // TODO Auto-generated method stub

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
        SmartDashboard.putBoolean("LED_getShooterOn", robotState.getShooterOn());
        SmartDashboard.putNumber("LED_getTargetShooterSpeed", robotState.getTargetShooterSpeed());
        SmartDashboard.putNumber("LED_getActualShooterSpeed", robotState.getActualShooterSpeed());
    }

    public void setLightsDisable() {
        for (int i = 0; i < m_LedBuffer.getLength(); i++) { 
            m_LedBuffer.setRGB(i, LED_DISABLE_R, LED_DISABLE_G, LED_DISABLE_B); 
        }
        m_led.setData(m_LedBuffer);

    }
    
    public void setLightsAuton() {
        for (int i = 0; i < m_LedBuffer.getLength(); i++) { 
            m_LedBuffer.setRGB(i, LED_AUTON_R, LED_AUTON_G, LED_AUTON_B); 
        }
        m_led.setData(m_LedBuffer);
    }

    public void setLightsTeleop() {
        if ((robotState.getShooterOn() == true) && (Math.abs(robotState.getTargetShooterSpeed() - robotState.getActualShooterSpeed()) < LED_SHOOTER_SPEED_THRESH)) {
            for (int i = 0; i < m_LedBuffer.getLength(); i++) { 
                m_LedBuffer.setRGB(i, LED_SHOOT_R, LED_SHOOT_G, LED_SHOOT_B); 
            }
        } else {
            for (int i = 0; i < m_LedBuffer.getLength(); i++) { 
                m_LedBuffer.setRGB(i, LED_TELEOP_R, LED_TELEOP_G, LED_TELEOP_B); 
            }
        }
        m_led.setData(m_LedBuffer);
    }
}