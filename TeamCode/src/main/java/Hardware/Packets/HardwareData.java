package Hardware.Packets;

import MathUtils.Vector4;

/**
 * Hardware Data Packet
 * Contains variables for hardware devices. Defaults all variables to 0
 */

public class HardwareData {
    private double bl, br, fl, fr, intakeLeft, intakeRight;
    private long timestamp;
    public HardwareData(){
        bl = 0;
        br = 0;
        fl = 0;
        fr = 0;
        intakeLeft = 0;
        intakeRight = 0;
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

    public double getIntakeRight() {
        return intakeRight;
    }

    public double getIntakeLeft() {
        return intakeLeft;
    }

    public void setIntakeRight(double intakeRight) {
        this.intakeRight = intakeRight;
    }

    public void setIntakeLeft(double intakeLeft) {
        this.intakeLeft = intakeLeft;
    }

    public void setIntakePowers(double powers){
        this.intakeRight = powers;
        this.intakeLeft = powers;
    }

    public long getTimestamp() {
        return timestamp;
    }

    public void setTimestamp(long timestamp) {
        this.timestamp = timestamp;
    }
}
