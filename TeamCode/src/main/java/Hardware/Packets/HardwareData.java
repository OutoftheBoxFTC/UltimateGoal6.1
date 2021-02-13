package Hardware.Packets;

import MathUtils.Vector4;

/**
 * Hardware Data Packet
 * Contains variables for hardware devices. Defaults all variables to 0
 */

public class HardwareData {
    private double bl, br, fl, fr, intake, shooter, shooterTilt, shooterLoadArm, wobbleLift, wobbleOneuseRight, wobbleLiftLeft, wobbleLiftRight;
    private long timestamp;
    public HardwareData(){
        bl = 0;
        br = 0;
        fl = 0;
        fr = 0;
        intake = 0;
        shooter = 0;
        wobbleLift = 0;
        wobbleOneuseRight = 0;
        wobbleLiftLeft = -1;
        wobbleLiftRight = -1;
    }

    public void setDriveMotors(Vector4 powers){
        fl = powers.getA();
        fr = powers.getB();
        bl = powers.getC();
        br = powers.getD();
    }

    public double getBl() {
        return bl;
    }

    public double getBr() {
        return br;
    }

    public double getFl() {
        return fl;
    }

    public double getFr() {
        return fr;
    }

    public double getIntake() {
        return intake;
    }

    public void setIntakePower(double powers){
        this.intake = powers;
    }

    public void setShooter(double shooter) {
        this.shooter = shooter;
    }

    public double getShooter() {
        return shooter;
    }

    public double getShooterTilt() {
        return shooterTilt;
    }

    public void setShooterTilt(double shooterTilt) {
        this.shooterTilt = shooterTilt;
    }

    public double getShooterLoadArm() {
        return shooterLoadArm;
    }

    public void setShooterLoadArm(double shooterLoadArm) {
        this.shooterLoadArm = shooterLoadArm;
    }

    public double getWobbleLift() {
        return wobbleLift;
    }

    public void setWobbleLift(double wobbleLift) {
        this.wobbleLift = wobbleLift;
    }

    public void setWobbleOneuseRight(double wobbleOneuseRight) {
        this.wobbleOneuseRight = wobbleOneuseRight;
    }

    public double getWobbleOneuseRight() {
        return wobbleOneuseRight;
    }

    public void setWobbleLiftLeft(double wobbleLiftLeft) {
        this.wobbleLiftLeft = wobbleLiftLeft;
    }

    public void setWobbleLiftRight(double wobbleLiftRight) {
        this.wobbleLiftRight = wobbleLiftRight;
    }

    public double getWobbleLiftLeft() {
        return wobbleLiftLeft;
    }

    public double getWobbleLiftRight() {
        return wobbleLiftRight;
    }

    public long getTimestamp() {
        return timestamp;
    }

    public void setTimestamp(long timestamp) {
        this.timestamp = timestamp;
    }
}
