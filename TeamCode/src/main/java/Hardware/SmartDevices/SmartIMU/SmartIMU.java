package Hardware.SmartDevices.SmartIMU;

import com.qualcomm.hardware.bosch.*;

import org.firstinspires.ftc.robotcore.external.navigation.*;

import Hardware.SmartDevices.*;

/**
 * Smart IMU
 * Class for computation and calculations to use the BNO055IMU
 */

public class SmartIMU extends SmartDevice {
    SmartIMUConfiguration configuration;
    final BNO055IMU imu;
    Orientation offset, position;
    public SmartIMU(BNO055IMU imu, SmartIMUConfiguration configuration){
        this.imu = imu;
        this.configuration = configuration;
        offset = new Orientation();
        position = new Orientation();
    }


    /**
     * Methods to get the three axis of motion
     * The result is calculated to restrict the rotation to between (0, 2pi) or (0, 360)
     */
    public double getHeading(){
        double tau = 2 * Math.PI;
        double rad = ((((position.firstAngle - offset.firstAngle) % tau) + tau) % tau);
        if(configuration.angleUnit == AngleUnit.RADIANS){
            return rad;
        }else{
            return Math.toDegrees(rad);
        }
    }

    public double getRoll(){
        double tau = 2 * Math.PI;
        double rad = ((((position.secondAngle - offset.secondAngle) % tau) + tau) % tau);
        if(configuration.angleUnit == AngleUnit.RADIANS){
            return rad;
        }else{
            return Math.toDegrees(rad);
        }
    }

    public double getPitch(){
        double tau = 2 * Math.PI;
        double rad = ((((position.thirdAngle - offset.thirdAngle) % tau) + tau) % tau);
        if(configuration.angleUnit == AngleUnit.RADIANS){
            return rad;
        }else{
            return Math.toDegrees(rad);
        }
    }

    @Override
    public void calibrate() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = configuration.sensorMode;
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        offset = imu.getAngularOrientation();
    }

    @Override
    public void update() {
        position = imu.getAngularOrientation();
    }

    public static enum AngleUnit{
        RADIANS,
        DEGREES;
    }
}
