package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;

public class RobotState {
    private Rotation2d rotation2d;
    private double tyReal;
    public double txReal;
    public double detecting;
    private boolean shooterReady;
    private boolean climberExtended;
    private double actualShooterSpeed;
    private double targetShooterSpeed;
    private boolean shooterOn;
    private boolean aimed;


    public RobotState() {
       rotation2d = new Rotation2d();
    }

    public boolean isShooterReady() {
        return shooterReady;
    }

    public void setShooterReady(boolean shooterReady) {
        this.shooterReady = shooterReady;
    }

    public Rotation2d getRotation2d() {
        return rotation2d;
    }

    public void setRotation2d(double degrees) {
        this.rotation2d = Rotation2d.fromDegrees(degrees);
    }

    public double getTheta() {
        return this.rotation2d.getDegrees();
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


    public void setIsAimed(boolean aimed){
        this.aimed = aimed;
    }

    public boolean getIsAimed(){
        return aimed;
    }

    public void setActualShooterSpeed(double speed) {
        this.actualShooterSpeed = speed;
    }

    public double getActualShooterSpeed() {
        return this.actualShooterSpeed;
    }

    public void setTargetShooterSpeed(double speed) {
        this.targetShooterSpeed = speed;
    }

    public double getTargetShooterSpeed() {
        return this.targetShooterSpeed;
    }

    public void setShooterOn(boolean shootOn) {
        this.shooterOn = shootOn;
    }

    public boolean getShooterOn() {
        return this.shooterOn;
    }
}
