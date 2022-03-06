package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;

public class RobotState {
    private double theta;
    private double tyReal;
    public double txReal;
    public double detecting;
    private boolean climberExtended;

    public void setTheta(double theta) {
        this.theta = theta;
    }

    public double getTheta() {
        return this.theta;
    }

    public void setClimberExtended(boolean climberExt) {
        this.climberExtended = climberExt;
    }

    public boolean getClimberExtended() {
        return this.climberExtended;
    }

    public void setTyReal(double tyReal) {
        this.tyReal = tyReal;
    }

    public double getTyReal() {
        return this.tyReal;
    }

    public void setTxReal(double txReal) {
        this.txReal = txReal;
    }

    public double getTxReal() {
        return this.txReal;
    }

    public void setDetecting(double detecting) {
        this.detecting = detecting;
    }

    public double getDetecting() {
        return this.detecting;
    }

    public double getTargetOffset() {
        return Math.pow(Math.abs(Math.sqrt(tyReal)) + Math.abs(Math.sqrt(txReal)), 2);
    }

    public boolean getWithinRange() {
        if (getTargetOffset() < Constants.ALLOWED_OFFSET) {
            return true;
        } else {
            return false;
        }
    }
}
