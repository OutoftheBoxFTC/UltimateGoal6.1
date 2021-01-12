package Hardware;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Hardware.Hardware;
import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import Hardware.SmartDevices.SmartEncoder.SmartEncoder;
import Hardware.SmartDevices.SmartEncoder.SmartEncoderConfiguration;
import Hardware.SmartDevices.SmartIMU.SmartIMU;
import Hardware.SmartDevices.SmartMotor.SmartMotor;
import Hardware.SmartDevices.SmartMotor.SmartMotorConfiguration;
import Hardware.SmartDevices.SmartNavXMicro.SmartNavXConfiguration;
import Hardware.SmartDevices.SmartNavXMicro.SmartNavXMicro;

public class UltimateGoalHardware extends Hardware {
    private long prevTime = 0;
    @Override
    public void registerDevices(HardwareMap map) {
        if(registeredDevices.contains(HardwareDevices.DRIVE_MOTORS)){
            smartDevices.put("Front Left", new SmartMotor(map.dcMotor.get("fl"), new SmartMotorConfiguration().reverseDirection()));
            smartDevices.put("Front Right", new SmartMotor(map.dcMotor.get("fr"), new SmartMotorConfiguration().reverseDirection()));
            smartDevices.put("Back Left", new SmartMotor(map.dcMotor.get("bl"), new SmartMotorConfiguration().reverseDirection()));
            smartDevices.put("Back Right", new SmartMotor(map.dcMotor.get("br"), new SmartMotorConfiguration().reverseDirection()));
        }
        if(registeredDevices.contains(HardwareDevices.ODOMETRY)){
            smartDevices.put("Odometry Left", new SmartEncoder(map.dcMotor.get("ol"), new SmartEncoderConfiguration()));
            smartDevices.put("Odometry Right", new SmartEncoder(map.dcMotor.get("or"), new SmartEncoderConfiguration().reverseDirection()));
            smartDevices.put("Odometry Aux", new SmartEncoder(map.dcMotor.get("oa"), new SmartEncoderConfiguration().reverseDirection()));
        }
        if(registeredDevices.contains(HardwareDevices.GYRO)){
            smartDevices.put("gyro", new SmartNavXMicro(map.get(NavxMicroNavigationSensor.class, "gyro"), new SmartNavXConfiguration().setAngleUnit(SmartIMU.AngleUnit.RADIANS)));
        }
    }

    @Override
    public void setHardware(HardwareData hardware) {
        if(enabledDevices.contains(HardwareDevices.DRIVE_MOTORS)){
            smartDevices.get("Front Left", SmartMotor.class).setPower(hardware.getFl());
            smartDevices.get("Front Right", SmartMotor.class).setPower(hardware.getFr());
            smartDevices.get("Back Left", SmartMotor.class).setPower(hardware.getBl());
            smartDevices.get("Back Right", SmartMotor.class).setPower(hardware.getBr());
        }
    }

    @Override
    public void setSensors(SensorData sensorData) {
        if(enabledDevices.contains(HardwareDevices.ODOMETRY)){
            sensorData.setOdometryLeft(smartDevices.get("Odometry Left", SmartEncoder.class).getCurrentPosition());
            sensorData.setOdometryRight(smartDevices.get("Odometry Right", SmartEncoder.class).getCurrentPosition());
            sensorData.setOdometryAux(smartDevices.get("Odometry Aux", SmartEncoder.class).getCurrentPosition());
        }
        if(enabledDevices.contains(HardwareDevices.GYRO)){
            sensorData.setGyro(smartDevices.get("gyro", SmartNavXMicro.class).getHeading());
        }
        sensorData.setFps(1/(MathUtils.MathUtils.nanoToDSec(System.nanoTime()-prevTime)));
        prevTime = System.nanoTime();
    }
}
