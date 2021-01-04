package Hardware.SmartDevices.SmartServo;

/**
 * Contains configuration methods for the Smart Servo class
 * Setting the init position sets the position the servo will be moved to when first powered
 */

public class SmartServoConfiguration {
    boolean direction;
    double initPos;
    public SmartServoConfiguration(){
        direction = false;
        initPos = 0;
    }

    public SmartServoConfiguration reverseDirection(){
        direction = true;
        return this;
    }

    public SmartServoConfiguration setInitPos(double pos){
        this.initPos = initPos;
        return this;
    }

    @Override
    public String toString() {
        return "SmartServoConfiguration{" +
                "direction=" + direction +
                ", initPos=" + initPos +
                '}';
    }
}
