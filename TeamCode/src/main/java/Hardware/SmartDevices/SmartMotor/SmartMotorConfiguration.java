package Hardware.SmartDevices.SmartMotor;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Smart Motor Configuration
 *
 * Contains configuration for the Smart Motor class
 */

public class SmartMotorConfiguration {
    boolean readPosition, readVelocity, direction;
    DcMotor.RunMode runMode;
    AngleUnit angleUnit;
    public SmartMotorConfiguration(){
        readPosition = false;
        readVelocity = false;
        runMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        angleUnit = AngleUnit.RADIANS;
    }

    public SmartMotorConfiguration readPosition(){
        this.readPosition = true;
        return this;
    }

    public SmartMotorConfiguration readVelocity(){
        readVelocity = true;
        return this;
    }

    public SmartMotorConfiguration setAngleUnit(AngleUnit angleUnit){
        this.angleUnit = angleUnit;
        return this;
    }

    public SmartMotorConfiguration setRunMode(DcMotor.RunMode runMode){
        this.runMode = runMode;
        return this;
    }

    public SmartMotorConfiguration reverseDirection(){
        this.direction = true;
        return this;
    }

    @Override
    public String toString() {
        return "SmartMotorConfiguration{" +
                "readPosition=" + readPosition +
                ", readVelocity=" + readVelocity +
                ", runMode=" + runMode +
                ", angleUnit=" + angleUnit +
                '}';
    }
}
