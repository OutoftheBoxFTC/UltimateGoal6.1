package Hardware.SmartDevices.SmartNavXMicro;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import Hardware.SmartDevices.SmartDevice;
import Hardware.SmartDevices.SmartIMU.SmartIMU;

public class SmartNavXMicro extends SmartDevice {
    private NavxMicroNavigationSensor navx;
    private Orientation angle, offset;
    private SmartNavXConfiguration configuration;

    public SmartNavXMicro(NavxMicroNavigationSensor navx, SmartNavXConfiguration smartNavXConfiguration){
        this.navx = navx;
        this.configuration = smartNavXConfiguration;
    }

    public double getHeading(){
        double tau = 2 * Math.PI;
        double rad = ((((angle.firstAngle - offset.firstAngle) % tau) + tau) % tau);
        if(configuration.angleUnit == SmartIMU.AngleUnit.RADIANS){
            return rad;
        }else{
            return Math.toDegrees(rad);
        }
    }

    public double getRoll(){
        double tau = 2 * Math.PI;
        double rad = ((((angle.secondAngle - offset.secondAngle) % tau) + tau) % tau);
        if(configuration.angleUnit == SmartIMU.AngleUnit.RADIANS){
            return rad;
        }else{
            return Math.toDegrees(rad);
        }
    }

    public double getPitch(){
        double tau = 2 * Math.PI;
        double rad = ((((angle.thirdAngle - offset.thirdAngle) % tau) + tau) % tau);
        if(configuration.angleUnit == SmartIMU.AngleUnit.RADIANS){
            return rad;
        }else{
            return Math.toDegrees(rad);
        }
    }

    @Override
    public void calibrate() {
        offset = navx.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
    }

    @Override
    public void update() {
        angle = navx.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
    }
}