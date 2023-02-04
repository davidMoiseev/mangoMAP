package frc.robot.subsystems;
import frc.robot.RobotCommander;
import frc.robot.RobotState;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.Constants.*;

public class Lights extends SubsystemBase{
    AddressableLED mainLed;
    AddressableLEDBuffer mainLedBuffer;

    private int r;
    private int g;
    private int b;

    private RobotState robotState;
    public Lights(RobotState robotState){
        
        this.robotState = robotState;

        mainLed = new AddressableLED(0);  //0
        mainLedBuffer = new AddressableLEDBuffer(25);  //26
        mainLed.setLength(mainLedBuffer.getLength());

        for (int i = 0; i < mainLedBuffer.getLength(); i++) {
           mainLedBuffer.setRGB(i, 0, 0, 0);
        }

        mainLed.setData(mainLedBuffer);
        mainLed.start();
       
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
        for (int i = 0; i < mainLedBuffer.getLength(); i++) {
            mainLedBuffer.setRGB(i, LED_DISABLE_R, LED_DISABLE_G, LED_DISABLE_B);
        }
        mainLed.setData(mainLedBuffer);
    }
    public void setLightsAuton() {
        for (int i = 0; i < mainLedBuffer.getLength(); i++) {
            mainLedBuffer.setRGB(i, LED_AUTON_R, LED_AUTON_G, LED_AUTON_B);
        }
        mainLed.setData(mainLedBuffer);
    }
    
    public void setLightsTeleop() {
        if ((robotState.getShooterOn() == true) && (Math.abs(robotState.getTargetShooterSpeed() - robotState.getActualShooterSpeed()) < LED_SHOOTER_SPEED_THRESH)) {
            r = LED_SHOOT_R;
            g = LED_SHOOT_G;
            b = LED_SHOOT_B;
        } else {
            r = LED_TELEOP_R;
            g = LED_TELEOP_G;
            b = LED_TELEOP_B;
        }

        for (int i = 8; i < mainLedBuffer.getLength() - 8; i++) {
            mainLedBuffer.setRGB(i, r, g, b);
        }


        if (robotState.getBallivatorTop() == true) {
            for (int i = 6; i < 8; i++) {
                mainLedBuffer.setRGB(i, SIDE_LED_BALL_R, SIDE_LED_BALL_G, SIDE_LED_BALL_B);
                mainLedBuffer.setRGB(i + 11, SIDE_LED_BALL_R, SIDE_LED_BALL_G, SIDE_LED_BALL_B);
            }
            for (int i = 0; i < 2; i++) {
                mainLedBuffer.setRGB(i, SIDE_LED_BALL_R, SIDE_LED_BALL_G, SIDE_LED_BALL_B);
                mainLedBuffer.setRGB(i + 23, SIDE_LED_BALL_R, SIDE_LED_BALL_G, SIDE_LED_BALL_B);
            }

        } else {
            for (int i = 6; i < 8; i++) {
                mainLedBuffer.setRGB(i, 0, 0, 0);
                mainLedBuffer.setRGB(i + 11, 0, 0, 0);
            }
            for (int i = 0; i < 2; i++) {
                mainLedBuffer.setRGB(i ,0, 0, 0);
                mainLedBuffer.setRGB(i + 23, 0, 0, 0);
            }
        }
        if (robotState.getBallivatorBottom() == true) {
            for (int i = 4; i < 6; i++) {
                mainLedBuffer.setRGB(i, SIDE_LED_BALL_R, SIDE_LED_BALL_G, SIDE_LED_BALL_B);
                mainLedBuffer.setRGB(i + 15, SIDE_LED_BALL_R, SIDE_LED_BALL_G, SIDE_LED_BALL_B);
            }
            for (int i = 2; i < 4; i++) {
                mainLedBuffer.setRGB(i, SIDE_LED_BALL_R, SIDE_LED_BALL_G, SIDE_LED_BALL_B);
                mainLedBuffer.setRGB(i + 19, SIDE_LED_BALL_R, SIDE_LED_BALL_G, SIDE_LED_BALL_B);
            }

        } else {
            for (int i = 4; i < 6; i++) {
                mainLedBuffer.setRGB(i, 0, 0, 0);
                mainLedBuffer.setRGB(i + 15, 0, 0, 0);
            }
            for (int i = 2; i < 4; i++) {
                mainLedBuffer.setRGB(i, 0, 0, 0);
                mainLedBuffer.setRGB(i + 19, 0, 0, 0);
            }

        }

        mainLed.setData(mainLedBuffer);

    }
}