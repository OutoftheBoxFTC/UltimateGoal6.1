package Hardware.SmartDevices.SmartMotor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.RobotLog;

import Hardware.SmartDevices.SmartDevice;

/**
 * Smart Motor
 * Contains methods to use the DCMotorEx class, which provides more functionality then DcMotor
 * Calibration sets the runmode and zeroes the encoder
 * If the set motor power is within 0.005 of the current motor power, it ignores the result to prevent unneeded setpower calls
 */

public class SmartMotor extends SmartDevice {
    private DcMotorEx motor;
    private SmartMotorConfiguration configuration;
    private volatile double position, velocity, power, prevPower, positionOffset;
    private int port;
    public SmartMotor(DcMotor motor, SmartMotorConfiguration configuration) {
        this.motor = (DcMotorEx)motor;
        this.configuration = configuration;
        power = 0;
        position = 0;
        velocity = 0;
        prevPower = 0;
        positionOffset = 0;
        motor.setMode(configuration.runMode);
        port = motor.getPortNumber();
    }

    public double getPosition() {
        return position - positionOffset;
    }

    public DcMotor getMotor() {
        return motor;
    }

    public double getPower() {
        return power;
    }

    public double getVelocity() {
        return velocity;
    }

    public void setPower(double power) {
        this.power = power;
    }

    @Override
    public String toString() {
        return "SmartMotor{" +
                "motor=" + getName() +
                ", configuration=" + configuration +
                ", position=" + position +
                ", velocity=" + velocity +
                ", power=" + power +
                '}';
    }

    @Override
    public void calibrate() {
        if(configuration.readPosition){
            positionOffset = (position * (configuration.direction ? -1 : 1));
        }
    }

    @Override
    public void update() {
        if(Math.abs(power - prevPower) > 0.005) {
            motor.setPower(power * (configuration.direction ? -1 : 1));
            prevPower = power;
        }
        if(configuration.readPosition){
            position = ((motor.getCurrentPosition() * (configuration.direction ? -1 : 1)) - positionOffset);
        }
        if(configuration.readVelocity){
            velocity = motor.getVelocity(configuration.angleUnit) * (configuration.direction ? -1 : 1);
        }
    }
}
