package Hardware.SmartDevices.SmartNavX2;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import Hardware.Drivers.NavX.ftc.AHRS;
import Hardware.SmartDevices.SmartDevice;
import Hardware.SmartDevices.SmartIMU.SmartIMU;
import Hardware.SmartDevices.SmartIMU.SmartIMUConfiguration;

public class SmartNavX2 extends SmartDevice {
    SmartIMUConfiguration configuration;
    final AHRS imu;
    Orientation offset, position;
    public SmartNavX2(int i2cPort, HardwareMap map, SmartIMUConfiguration configuration){
        this.imu = AHRS.getInstance(map.deviceInterfaceModule.get("dim"), i2cPort, AHRS.DeviceDataType.kProcessedData);
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
        if(configuration.angleUnit == SmartIMU.AngleUnit.RADIANS){
            return rad;
        }else{
            return Math.toDegrees(rad);
        }
    }

    public double getRoll(){
        double tau = 2 * Math.PI;
        double rad = ((((position.secondAngle - offset.secondAngle) % tau) + tau) % tau);
        if(configuration.angleUnit == SmartIMU.AngleUnit.RADIANS){
            return rad;
        }else{
            return Math.toDegrees(rad);
        }
    }

    public double getPitch(){
        double tau = 2 * Math.PI;
        double rad = ((((position.thirdAngle - offset.thirdAngle) % tau) + tau) % tau);
        if(configuration.angleUnit == SmartIMU.AngleUnit.RADIANS){
            return rad;
        }else{
            return Math.toDegrees(rad);
        }
    }

    @Override
    public void calibrate() {
        if(imu.isConnected()){
            offset = new Orientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS, imu.getYaw(), imu.getRoll(), imu.getPitch(), 0);
        }
    }

    @Override
    public void stop(){
        imu.close();
    }

    @Override
    public void update() {
        position = new Orientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS, imu.getYaw(), imu.getRoll(), imu.getPitch(), 0);
    }

    public static enum AngleUnit{
        RADIANS,
        DEGREES;
    }
}
