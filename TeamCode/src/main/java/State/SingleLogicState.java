package State;

import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;

/**
 * A single logic state runs main() once before de-activating itself
 * Best used in the EventSystem with triggers running close to every frame, or run several times
 */

public abstract class SingleLogicState extends LogicState {
    public SingleLogicState(StateMachine stateMachine) {
        super(stateMachine);
    }

    public abstract void run(SensorData sensorData, HardwareData hardwareData);

    @Override
    public void update(SensorData sensorData, HardwareData hardwareData) {
        run(sensorData, hardwareData);
        deactivateThis();
    }
}
