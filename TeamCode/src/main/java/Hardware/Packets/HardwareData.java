package Hardware.Packets;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import MathSystems.Vector4;

/**
 * Hardware Data Packet
 * Contains variables for hardware devices. Defaults all variables to 0
 */

public class HardwareData {
    private double bl, br, fl, fr, intake, shooter, shooterTilt, shooterLoadArm, wobbleLift, wobbleOneuseRight, wobbleLiftLeft, wobbleLiftRight, intakeRelease, turret, intakeShield;
    private long timestamp;
    private RevBlinkinLedDriver.BlinkinPattern pattern;
    public HardwareData(){
        bl = 0;
        br = 0;
        fl = 0;
        fr = 0;
        intake = 0;
        shooter = 0;
        wobbleLift = 0;
        wobbleOneuseRight = -1;
        wobbleLiftLeft = -1;
        wobbleLiftRight = -1;
        shooterLoadArm = -1;
        intakeRelease = -1;
        wobbleLift = 0;
        turret = -1;
        intakeShield = -1;
        //pattern = RevBlinkinLedDriver.BlinkinPattern.CP1_2_TWINKLES;
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

    public void setIntakeRelease(double intakeRelease) {
        this.intakeRelease = intakeRelease;
    }

    public double getIntakeRelease() {
        return intakeRelease;
    }

    public void setTurret(double turret) {
        this.turret = turret;
    }

    public double getTurret() {
        return turret;
    }

    public RevBlinkinLedDriver.BlinkinPattern getPattern() {
        return pattern;
    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        this.pattern = pattern;
    }

    public void setIntakeShield(double intakeShield) {
        this.intakeShield = intakeShield;
    }

    public double getIntakeShield() {
        return intakeShield;
    }

    public long getTimestamp() {
        return timestamp;
    }

    public void setTimestamp(long timestamp) {
        this.timestamp = timestamp;
    }
}
