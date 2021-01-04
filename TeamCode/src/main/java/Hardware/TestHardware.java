package Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import Hardware.SmartDevices.SmartVuforia.SmartPhoneVuforia;

/**
 * Test Hardware class, which is completely empty
 * This can be used to test classes without attaching the phone to the robot
 */

public class TestHardware extends Hardware {

    @Override
    public void registerDevices(HardwareMap map) {

    }

    @Override
    public void setHardware(HardwareData hardware) {

    }

    @Override
    public void setSensors(SensorData sensorData) {

    }
}
