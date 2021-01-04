package Hardware.SmartDevices.SmartServo;

import com.qualcomm.robotcore.hardware.*;

import Hardware.SmartDevices.*;

/**
 * Contains methods for the usage of the Servo and ServoImpl classes
 * Allows for the enabling and disabling of the PMW to the servo, which will stop it from holding position
 */

public class SmartServo extends SmartDevice {
    private Servo servo;
    private double position, prevPosition;
    private int port;
    private SmartServoConfiguration configuration;
    public SmartServo(Servo servo, SmartServoConfiguration configuration) {
        this.servo = servo;
        port = servo.getPortNumber();
        this.configuration = configuration;
        this.position = configuration.initPos;
        this.prevPosition = 0;
    }

    public double getPosition() {
        return position;
    }

    public void setPosition(double position) {
        this.position = position;
    }

    public Servo getServo() {
        return servo;
    }

    public void enableServo(){
        ServoImplEx servoImpl = (ServoImplEx)servo;
        if(!servoImpl.isPwmEnabled()){
            servoImpl.setPwmEnable();
        }
    }

    public void disableServo(){
        ServoImplEx servoImpl = (ServoImplEx)servo;
        if(servoImpl.isPwmEnabled()){
            servoImpl.setPwmDisable();
        }
    }

    @Override
    public void calibrate() {
        if(configuration.direction){
            servo.setDirection(Servo.Direction.REVERSE);
        }
    }

    @Override
    public void update() {
        if(Math.abs(position - prevPosition) < 0.005 && ((ServoImplEx)servo).isPwmEnabled()){
            servo.setPosition(position);
            prevPosition = position;
        }
    }
}
