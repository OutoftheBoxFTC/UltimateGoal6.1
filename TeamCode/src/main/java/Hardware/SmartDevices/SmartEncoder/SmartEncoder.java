package Hardware.SmartDevices.SmartEncoder;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.RobotLog;

import Hardware.SmartDevices.*;

/**
 * Smart Encoder
 * Attaches itself to a DcMotor and reads the position. This method is similar to using
 * the DcMotor with the getPosition, but the smart encoder does not set power
 * Use in conjunction with a SmartMotor to increase readability
 */

public class SmartEncoder extends SmartDevice {
    private volatile int position;
    private volatile int offset;
    private SmartEncoderConfiguration configuration;
    private DcMotor motor;

    public SmartEncoder(DcMotor motor, SmartEncoderConfiguration configuration){
        this.motor = motor;
        this.configuration = configuration;
    }

    @Override
    public void calibrate() {
        offset = motor.getCurrentPosition();
    }

    public int getCurrentPosition(){
        return position;
    }

    @Override
    public void update() {
        position = (motor.getCurrentPosition() - offset) * (configuration.direction ? -1 : 1);
    }
}
