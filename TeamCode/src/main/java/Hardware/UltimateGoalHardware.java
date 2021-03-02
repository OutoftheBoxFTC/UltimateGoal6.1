package Hardware;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import Hardware.SmartDevices.SmartCV.SmartCV;
import Hardware.SmartDevices.SmartEncoder.SmartEncoder;
import Hardware.SmartDevices.SmartEncoder.SmartEncoderConfiguration;
import Hardware.SmartDevices.SmartIMU.SmartIMU;
import Hardware.SmartDevices.SmartMotor.SmartMotor;
import Hardware.SmartDevices.SmartMotor.SmartMotorConfiguration;
import Hardware.SmartDevices.SmartNavXMicro.SmartNavXConfiguration;
import Hardware.SmartDevices.SmartNavXMicro.SmartNavXMicro;
import Hardware.SmartDevices.SmartServo.SmartServo;
import Hardware.SmartDevices.SmartServo.SmartServoConfiguration;
import Hardware.SmartDevices.SmartTensorflow.SmartTensorflow;

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
            smartDevices.put("Odometry Left", new SmartEncoder(map.dcMotor.get("intake"), new SmartEncoderConfiguration().reverseDirection()));
            smartDevices.put("Odometry Right", new SmartEncoder(map.dcMotor.get("br"), new SmartEncoderConfiguration()));
            smartDevices.put("Odometry Aux", new SmartEncoder(map.dcMotor.get("oa"), new SmartEncoderConfiguration()));
        }
        if(registeredDevices.contains(HardwareDevices.INTAKE)){
            smartDevices.put("Intake", new SmartMotor(map.dcMotor.get("intake"), new SmartMotorConfiguration()));
            smartDevices.put("Intake Release", new SmartServo(map.servo.get("intakeRelease"), new SmartServoConfiguration()));
        }
        if(registeredDevices.contains(HardwareDevices.GYRO)){
            //smartDevices.put("gyro", new SmartNavXMicro(map.get(NavxMicroNavigationSensor.class, "gyro"), new SmartNavXConfiguration().setAngleUnit(SmartIMU.AngleUnit.RADIANS)));
        }
        if(registeredDevices.contains(HardwareDevices.SHOOTER)){
            smartDevices.put("Shooter Left", new SmartMotor(map.dcMotor.get("ol"), new SmartMotorConfiguration().readVelocity()));
            smartDevices.put("Shooter Right", new SmartMotor(map.dcMotor.get("shooter"), new SmartMotorConfiguration().readVelocity()));
            smartDevices.put("Shooter Tilt", new SmartServo(map.servo.get("shooterTilt"), new SmartServoConfiguration().setInitPos(0.37)));
            smartDevices.put("Shooter Indexer", new SmartServo(map.servo.get("shooterLoadArm"), new SmartServoConfiguration().setInitPos(0.8)));
        }
        if(registeredDevices.contains(HardwareDevices.WOBBLE)){
            smartDevices.put("Wobble Oneuse Right", new SmartServo(map.servo.get("wobbleR"), new SmartServoConfiguration().setInitPos(0.5)));
            smartDevices.put("Wobble Lift Right", new SmartServo(map.servo.get("wobbleLR"), new SmartServoConfiguration().setInitPos(0.5)));
            smartDevices.put("Wobble Lift Left", new SmartServo(map.servo.get("wobbleLL"), new SmartServoConfiguration().setInitPos(0.5)));
            smartDevices.put("Wobble Lift", new SmartMotor(map.dcMotor.get("wobbleLift"), new SmartMotorConfiguration().readPosition()));
        }
        if(registeredDevices.contains(HardwareDevices.WEBCAM)){
            smartDevices.put("Ring Detector", new SmartCV(map.get(WebcamName.class, "Webcam 1"), map));
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
        if(enabledDevices.contains(HardwareDevices.WOBBLE)){
            smartDevices.get("Wobble Oneuse Right", SmartServo.class).setPosition(hardware.getWobbleOneuseRight());
            smartDevices.get("Wobble Lift Right", SmartServo.class).setPosition(hardware.getWobbleLiftRight());
            smartDevices.get("Wobble Lift Left", SmartServo.class).setPosition(hardware.getWobbleLiftLeft());
            smartDevices.get("Wobble Lift", SmartMotor.class).setPower(hardware.getWobbleLift());
        }
        if(enabledDevices.contains(HardwareDevices.INTAKE)){
            smartDevices.get("Intake", SmartMotor.class).setPower(hardware.getIntake());
            smartDevices.get("Intake Release", SmartServo.class).setPosition(hardware.getIntakeRelease());
        }
        if(enabledDevices.contains(HardwareDevices.SHOOTER)){
            smartDevices.get("Shooter Left", SmartMotor.class).setPower(hardware.getShooter());
            smartDevices.get("Shooter Right", SmartMotor.class).setPower(hardware.getShooter());
            smartDevices.get("Shooter Tilt", SmartServo.class).setPosition(hardware.getShooterTilt());
            smartDevices.get("Shooter Indexer", SmartServo.class).setPosition(hardware.getShooterLoadArm());
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
            //sensorData.setGyro(smartDevices.get("gyro", SmartNavXMicro.class).getHeading());
        }
        if(enabledDevices.contains(HardwareDevices.WEBCAM)){
            sensorData.setRings(smartDevices.get("Ring Detector", SmartCV.class).getRings());
        }
        if(enabledDevices.contains(HardwareDevices.WOBBLE)){
            sensorData.setWobbleLift(smartDevices.get("Wobble Lift", SmartMotor.class).getPosition());
        }
        sensorData.setFps(1/(MathSystems.MathUtils.nanoToDSec(System.nanoTime()-prevTime)));
        prevTime = System.nanoTime();
    }
}
