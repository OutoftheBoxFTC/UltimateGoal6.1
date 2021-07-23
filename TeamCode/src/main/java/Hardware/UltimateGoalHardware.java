package Hardware;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import Hardware.SmartDevices.Smart2MSensor.Smart2MSensor;
import Hardware.SmartDevices.Smart2MSensor.Smart2MSensorConfiguration;
import Hardware.SmartDevices.SmartBlinkin.SmartBlinkin;
import Hardware.SmartDevices.SmartCV.SmartCV;
import Hardware.SmartDevices.SmartEncoder.SmartEncoder;
import Hardware.SmartDevices.SmartEncoder.SmartEncoderConfiguration;
import Hardware.SmartDevices.SmartMotor.SmartMotor;
import Hardware.SmartDevices.SmartMotor.SmartMotorConfiguration;
import Hardware.SmartDevices.SmartServo.SmartServo;
import Hardware.SmartDevices.SmartServo.SmartServoConfiguration;

public class UltimateGoalHardware extends Hardware {
    private long prevTime = 0;
    @Override
    public void registerDevices(HardwareMap map) {
        if(registeredDevices.contains(HardwareDevices.DRIVE_MOTORS)){
            smartDevices.put("Front Left", new SmartMotor(map.dcMotor.get("fl"), new SmartMotorConfiguration().reverseDirection()));
            smartDevices.put("Front Right", new SmartMotor(map.dcMotor.get("fr"), new SmartMotorConfiguration().reverseDirection()));
            smartDevices.put("Back Left", new SmartMotor(map.dcMotor.get("bl"), new SmartMotorConfiguration().reverseDirection().readPosition()));
            smartDevices.put("Back Right", new SmartMotor(map.dcMotor.get("br"), new SmartMotorConfiguration().reverseDirection()));
        }
        if(registeredDevices.contains(HardwareDevices.ODOMETRY)){
            smartDevices.put("Odometry Left", new SmartEncoder(map.dcMotor.get("intake"), new SmartEncoderConfiguration().reverseDirection()));
            smartDevices.put("Odometry Right", new SmartEncoder(map.dcMotor.get("br"), new SmartEncoderConfiguration()));
            smartDevices.put("Odometry Aux", new SmartEncoder(map.dcMotor.get("oa"), new SmartEncoderConfiguration()));
        }
        if(registeredDevices.contains(HardwareDevices.INTAKE)){
            smartDevices.put("Intake", new SmartMotor(map.dcMotor.get("intake"), new SmartMotorConfiguration()));
            smartDevices.put("Intake Top", new SmartMotor(map.dcMotor.get("oa"), new SmartMotorConfiguration().reverseDirection()));
            smartDevices.put("Intake Shield", new SmartServo(map.servo.get("intakeShield"), new SmartServoConfiguration()));
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
            smartDevices.put("Wobble Oneuse Left", new SmartServo(map.servo.get("wobbleL"), new SmartServoConfiguration().setInitPos(0.5)));
            smartDevices.put("Wobble Fourbar Right", new SmartServo(map.servo.get("wobbleVL"), new SmartServoConfiguration().setInitPos(0.5)));
            smartDevices.put("Wobble Fourbar Left", new SmartServo(map.servo.get("wobbleVR"), new SmartServoConfiguration().setInitPos(0.5)));
            smartDevices.put("Wobble Fork Left", new SmartServo(map.servo.get("wobbleFR"), new SmartServoConfiguration()));
            smartDevices.put("Wobble Fork Right", new SmartServo(map.servo.get("wobbleFL"), new SmartServoConfiguration()));
        }
        if(registeredDevices.contains(HardwareDevices.WEBCAM)){
            //smartDevices.put("Ring Detector", new SmartCV(map.get(WebcamName.class, "Webcam 2"), map));
            //smartDevices.put("TowerCam", new TowerCV(map.get(WebcamName.class, "Webcam 1"), map));
            smartDevices.put("SmartCV", new SmartCV(map.get(WebcamName.class, "Webcam 3"), map.get(WebcamName.class, "Webcam 2"), map.get(WebcamName.class, "Webcam 1"), map));
        }
        if(registeredDevices.contains(HardwareDevices.TURRET)){
            smartDevices.put("Turret", new SmartServo(map.servo.get("turret"), new SmartServoConfiguration().setInitPos(0.5)));
        }
        if(registeredDevices.contains(HardwareDevices.LEDS)){
            smartDevices.put("Blinkin", new SmartBlinkin(map.get(RevBlinkinLedDriver.class, "Blinkin")));
        }
        if(registeredDevices.contains(HardwareDevices.DISTANCE_SENSOR)){
            smartDevices.put("Distance Sensor", new Smart2MSensor(map.get(Rev2mDistanceSensor.class, "distanceSensor"), new Smart2MSensorConfiguration().setUnit(DistanceUnit.INCH)));
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
            smartDevices.get("Wobble Oneuse Left", SmartServo.class).setPosition(hardware.getWobbleOneuseLeft());
            smartDevices.get("Wobble Fourbar Right", SmartServo.class).setPosition(hardware.getWobbleFourbarRight());
            smartDevices.get("Wobble Fourbar Left", SmartServo.class).setPosition(hardware.getWobbleFourbarLeft());
            smartDevices.get("Wobble Fork Left", SmartServo.class).setPosition(hardware.getWobbleForkLeft());
            smartDevices.get("Wobble Fork Right", SmartServo.class).setPosition(hardware.getWobbleForkRight());
        }
        if(enabledDevices.contains(HardwareDevices.INTAKE)){
            smartDevices.get("Intake", SmartMotor.class).setPower(hardware.getIntake());
            smartDevices.get("Intake Top", SmartMotor.class).setPower(hardware.getIntake());
            smartDevices.get("Intake Shield", SmartServo.class).setPosition(hardware.getIntakeShield());
        }
        if(enabledDevices.contains(HardwareDevices.SHOOTER)){
            smartDevices.get("Shooter Left", SmartMotor.class).setPower(hardware.getShooter());
            smartDevices.get("Shooter Right", SmartMotor.class).setPower(hardware.getShooter());
            smartDevices.get("Shooter Tilt", SmartServo.class).setPosition(hardware.getShooterTilt());
            smartDevices.get("Shooter Indexer", SmartServo.class).setPosition(hardware.getShooterLoadArm());
        }
        if(enabledDevices.contains(HardwareDevices.TURRET)){
            smartDevices.get("Turret", SmartServo.class).setPosition(hardware.getTurret());
        }
        if(enabledDevices.contains(HardwareDevices.LEDS)){
            smartDevices.get("Blinkin", SmartBlinkin.class).setPattern(hardware.getPattern());
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
            SmartCV smartCV = smartDevices.get("SmartCV", SmartCV.class);
            sensorData.setRings(smartCV.getRings());
            sensorData.setTrack(smartCV.getTrack());
            sensorData.setHeading(smartCV.getRedHeading());
            sensorData.setRange(smartCV.getRange());
            sensorData.setPitch(smartCV.getPitch());
            sensorData.setPowershots(smartCV.getRedPowershots());
        }
        if(enabledDevices.contains(HardwareDevices.DISTANCE_SENSOR)){
            sensorData.setDistance(smartDevices.get("Distance Sensor", Smart2MSensor.class).getDistance());
        }
        sensorData.setFps(1/(MathSystems.MathUtils.nanoToDSec(System.nanoTime()-prevTime)));
        prevTime = System.nanoTime();
    }
}
