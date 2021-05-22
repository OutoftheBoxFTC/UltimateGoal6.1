package OpModes.Debug;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Hardware.*;
import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import Hardware.SmartDevices.SmartCV.SmartCV;
import OpModes.BasicOpmode;
import State.LogicState;

@TeleOp
@Disabled
public class RingVisionTest extends BasicOpmode {
    public RingVisionTest() {
        super(new UltimateGoalHardware());
    }

    @Override
    public void setup() {
        hardware.enableAll();
        hardware.registerAll();

        eventSystem.onInit("Init", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                telemetry.addData("Rings", sensorData.getRings());

                if(isStarted()){
                    deactivateThis();
                }
            }
        });

        eventSystem.onStart("Start", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                telemetry.addData("Range", sensorData.getRange());
            }
        });
    }
}
