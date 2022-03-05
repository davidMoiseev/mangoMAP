package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;

public class RobotState {
    private double theta;
    private double tyReal;
    public double txReal;
    public double detecting;
    private double DetectingDelay;

    public void setTheta(double theta){
        this.theta = theta;
    }

    public double getTheta(){
        return this.theta;
    }

    public void setTyReal(double tyReal){
        this.tyReal = tyReal;
    }

    public double getTyReal(){
        return this.tyReal;
    }

    public void setTxReal(double txReal){
        this.txReal = txReal;
    }

    public double getTxReal(){
        return this.txReal;
    }

    public void setDetecting(double detecting){
        this.detecting = detecting;
    }

    public double getDetecting(){
        return this.detecting;
    }

    public void setDetectingDelay(double DetectingDelay){
        this.DetectingDelay = DetectingDelay;
    }

    public double getDetectingDelay(){
        return this.DetectingDelay;
    }


}
