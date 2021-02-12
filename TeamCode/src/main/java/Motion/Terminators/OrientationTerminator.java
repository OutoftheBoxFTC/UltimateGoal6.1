package Motion.Terminators;

import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import MathUtils.MathUtils;
import MathUtils.Vector3;

/**
 * Orientation Terminator
 * Creates a terminator that terminates when the position gets within a specified distance of the target
 */

public class OrientationTerminator extends Terminator {
    private Vector3 position, target;
    private double distance, rotation;

    public OrientationTerminator(Vector3 position, Vector3 target, double distance, double rotation){
        this.position = position;
        this.target = target;
        this.distance = distance;
        this.rotation = rotation;
    }

    @Override
    public boolean shouldTerminate(SensorData sensorData, HardwareData hardwareData) {
        return (position.getVector2().distanceTo(target.getVector2())) < distance && Math.abs(MathUtils.getRadRotDist(position.getC(), target.getC())) < Math.toRadians(rotation);
    }
}
