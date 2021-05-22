package OpModes.Debug;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Hardware.*;
import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import OpModes.BasicOpmode;
import State.LogicState;

@TeleOp
public class CleanIntakeWheels extends BasicOpmode {
    public CleanIntakeWheels() {
        super(new UltimateGoalHardware());
    }

    @Override
    public void setup() {
        hardware.registerDevice(Hardware.HardwareDevices.INTAKE);
        hardware.enableDevice(Hardware.HardwareDevices.INTAKE);

        eventSystem.onStart("Spin", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                hardwareData.setIntakePower(-1);
            }
        });
    }
}
