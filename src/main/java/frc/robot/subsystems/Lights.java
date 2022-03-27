package frc.robot.subsystems;

import frc.robot.RobotCommander;
import frc.robot.RobotState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.Constants.*;

import java.util.Random;

public class Lights extends SubsystemBase{
    AddressableLED m_led;
    AddressableLEDBuffer m_LedBuffer;
    private RobotState robotState;
	private int autonCycles;
	private Alliance team;

     public Lights(RobotState robotState){
       this.robotState = robotState;
       m_led = new AddressableLED(0);
       m_LedBuffer = new AddressableLEDBuffer(26);
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

    }

    public void setLightsDisable() {
		autonCycles = 0;
		boolean runFancyDisable = SmartDashboard.getBoolean("fancyDisable", true);

		if (runFancyDisable) {
			fancyDisable();
		} else {
			for (int i = 0; i < m_LedBuffer.getLength(); i++) {
            	m_LedBuffer.setRGB(i, LED_DISABLE_R, LED_DISABLE_G, LED_DISABLE_B);
			}
			m_led.setData(m_LedBuffer);	
		}
    }

	private void fancyDisable() {

		Random mya = new Random();
		int maxChange = 8;
		int lightNum = mya.nextInt(maxChange);
		for (int f = 0; f < lightNum; f++) {

			Random greenPM = new Random();
			int upperboundG = 19;
			int greenDev = greenPM.nextInt(upperboundG);
	
			Random redPM = new Random();
			int upperboundR = 13;
			int redDev = redPM.nextInt(upperboundR);
	
			Random randLight = new Random();
			int maxLight = 26;//number of addressable leds
			int lightDev = randLight.nextInt(maxLight);

			m_LedBuffer.setRGB(lightDev, (255 - (redDev * 4)), (25 + (greenDev * 4)), 0);
		}
		
		m_led.setData(m_LedBuffer);	
	}

    public void setLightsAuton() {

		if (autonCycles < 10) {
			team = DriverStation.getAlliance();
			autonCycles++;
		}
		
		if (team == Alliance.Blue) {
			for (int i = 0; i < m_LedBuffer.getLength(); i++) {
				m_LedBuffer.setRGB(i, 0, 150, 255);
			}
			m_led.setData(m_LedBuffer);
		} else if (team == Alliance.Red) {
			for (int i = 0; i < m_LedBuffer.getLength(); i++) {
				m_LedBuffer.setRGB(i, 255, 100, 0);
			}
			m_led.setData(m_LedBuffer);
		}
    }

    public void setLightsTeleop() {
		
		double shooterError = Math.abs(robotState.getTargetShooterSpeed() - robotState.getActualShooterSpeed());
        double targeting = robotState.getDetecting();
        boolean shooterActive;

        if (robotState.getActualShooterSpeed() > 50) {
			shooterActive = true;
		} else {
			shooterActive = false;
		}
		
		if ((shooterActive == true) && (shooterError < LED_SHOOTER_SPEED_THRESH)) { // if shooter is on and within threshold, turn lights {green}
		
			    for (int i = 0; i < m_LedBuffer.getLength(); i++) {
					m_LedBuffer.setRGB(i, LED_SHOOT_R, LED_SHOOT_G, LED_SHOOT_B);
            }
			        m_led.setData(m_LedBuffer);
		} else if ((shooterActive == true) && (Math.abs(robotState.getTargetShooterSpeed()- robotState.getActualShooterSpeed()) >= LED_SHOOTER_SPEED_THRESH)) { // if shooter is on but not within threshold

			if (shooterError - LED_SHOOTER_SPEED_THRESH < 255)  { // if shooter is under 255 rpm away from the threshold, then start fading lights red lights off based on rpm
				for (int i = 0; i < m_LedBuffer.getLength(); i++) {
					m_LedBuffer.setRGB(i, (int) (255 - (shooterError - (LED_SHOOTER_SPEED_THRESH + 255))), 255, 0);
				}
				m_led.setData(m_LedBuffer);
				
			} else if ((shooterError - LED_SHOOTER_SPEED_THRESH) < 510){ // if shooter is under 510 rpm away from the threshold, then start fading green lights on based on rpm
				for (int i = 0; i < m_LedBuffer.getLength(); i++) {
					m_LedBuffer.setRGB(i, 255, (int) (shooterError - (LED_SHOOTER_SPEED_THRESH + 255)), 0);
				}	
				m_led.setData(m_LedBuffer);		
				
			} else if (shooterError > 510 + LED_SHOOTER_SPEED_THRESH) { // if shooter error is more than 510rpm + [250rpm] (LED_SHOOTER_SPEED_THRESH) away from shooter target,then turn lights red
				for (int i = 0; i < m_LedBuffer.getLength(); i++) {
					m_LedBuffer.setRGB(i, 255, 0, 0);
				}	
				m_led.setData(m_LedBuffer);

			}
		} else if ((targeting != 0) && (shooterActive == false)) { // if the limelight is targeting, and the shooter isnt spinning up, make the lights [magenta]
			for (int i = 0; i < m_LedBuffer.getLength(); i++) {
				m_LedBuffer.setRGB(i, LED_DETECT_R, LED_DETECT_G, LED_DETECT_B);
			}
			m_led.setData(m_LedBuffer);
		
		}  else if (shooterActive == false)  {
		 	for (int i = 0; i < m_LedBuffer.getLength(); i++) { // if shooter isnt on, then turn to teleop lights
		 		m_LedBuffer.setRGB(i, LED_TELEOP_R, LED_TELEOP_G, LED_TELEOP_B);
		 	}
		 	m_led.setData(m_LedBuffer);
		 }
	}
}
