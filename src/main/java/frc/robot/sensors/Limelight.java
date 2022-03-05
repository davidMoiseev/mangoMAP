package frc.robot.sensors;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotCommander;
import frc.robot.RobotState;

    
public class Limelight extends SensorBase{

    private double txReal = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    private double tyReal = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    private double Detecting = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);

    public Limelight(RobotState robotState) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

        NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");
  
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);
  
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(6);
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    }

    @Override
    public void updateState(RobotState robotState, RobotCommander commander) {
        txReal = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        tyReal = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        Detecting = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);

        robotState.setDetecting(Detecting);
        robotState.setTyReal(tyReal);
        robotState.setTxReal(txReal);
    }

    @Override
    public void zeroSensor() {
        
    }

    @Override
    public void logData() {

    }
    
}
