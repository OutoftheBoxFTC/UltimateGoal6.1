package Hardware.SmartDevices.SmartNavXMicro;

import com.qualcomm.hardware.bosch.BNO055IMU;

import Hardware.SmartDevices.SmartIMU.SmartIMU;

/**
 * Smart IMU Configuration
 * Contains configurations for setting the Angle Unit and Sensor Mode
 */

public class SmartNavXConfiguration {
    SmartIMU.AngleUnit angleUnit;
    public SmartNavXConfiguration(){
        angleUnit = SmartIMU.AngleUnit.DEGREES;
    }

    public SmartNavXConfiguration setAngleUnit(SmartIMU.AngleUnit angleUnit) {
        this.angleUnit = angleUnit;
        return this;
    }

    @Override
    public String toString() {
        return "SmartIMUConfiguration{" +
                "angleUnit=" + angleUnit +
                '}';
    }
}