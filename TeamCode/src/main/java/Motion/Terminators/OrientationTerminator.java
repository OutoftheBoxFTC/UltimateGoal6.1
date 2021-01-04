package Motion.Terminators;

import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import MathUtils.Vector3;

/**
 * Orientation Terminator
 * Creates a terminator that terminates when the position gets within a specified distance of the target
 */

public class OrientationTerminator extends Terminator {
    private Vector3 position, target;
    private double distance;

    public OrientationTerminator(Vector3 position, Vector3 target, double distance){
        this.position = position;
        this.target = target;
        this.distance = distance;
    }

    @Override
    public boolean shouldTerminate(SensorData sensorData, HardwareData hardwareData) {
        return (position.getVector2().distanceTo(target.getVector2())) < distance;
    }
}
