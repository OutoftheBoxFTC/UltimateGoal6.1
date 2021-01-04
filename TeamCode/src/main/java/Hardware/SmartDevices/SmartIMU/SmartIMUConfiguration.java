package Hardware.SmartDevices.SmartIMU;

import com.qualcomm.hardware.bosch.BNO055IMU;

/**
 * Smart IMU Configuration
 * Contains configurations for setting the Angle Unit and Sensor Mode
 */

public class SmartIMUConfiguration {
    SmartIMU.AngleUnit angleUnit;
    BNO055IMU.SensorMode sensorMode;
    public SmartIMUConfiguration(){
        angleUnit = SmartIMU.AngleUnit.DEGREES;
        sensorMode = BNO055IMU.SensorMode.IMU;
    }

    public SmartIMUConfiguration setAngleUnit(SmartIMU.AngleUnit angleUnit) {
        this.angleUnit = angleUnit;
        return this;
    }

    public SmartIMUConfiguration setSensorMode(BNO055IMU.SensorMode sensorMode) {
        this.sensorMode = sensorMode;
        return this;
    }

    @Override
    public String toString() {
        return "SmartIMUConfiguration{" +
                "angleUnit=" + angleUnit +
                ", sensorMode=" + sensorMode +
                '}';
    }
}
