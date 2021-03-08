package Motion.Terminators;

import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import MathSystems.MathUtils;
import MathSystems.Vector2;
import MathSystems.Vector3;

/**
 * Orientation Terminator
 * Creates a terminator that terminates when the position gets within a specified distance of the target
 */

public class OrientationTerminator extends Terminator {
    private Vector3 position, target;
    private double distance, rotation;
    private int frameCount = 0, numFrames;

    public OrientationTerminator(Vector3 position, Vector3 target, double distance, double rotation){
        this.position = position;
        this.target = target;
        this.distance = distance;
        this.rotation = rotation;
        numFrames = 0;
    }

    public OrientationTerminator(Vector3 position, Vector3 target){
        this.position = position;
        this.target = target;
        this.distance = 1;
        this.rotation = 1;
    }

    public OrientationTerminator(Vector3 position, Vector2 target){
        this.position = position;
        this.target = target.toVector3(0);
        this.distance = 1;
        this.rotation = Double.MAX_VALUE;
        numFrames = 0;
    }

    public OrientationTerminator(Vector3 position, Vector2 target, double distance){
        this.position = position;
        this.target = target.toVector3(0);
        this.distance = distance;
        this.rotation = Double.MAX_VALUE;
        numFrames = 0;
    }

    public OrientationTerminator(Vector3 position, Vector3 target, double distance, double rotation, int numFrames){
        this.position = position;
        this.target = target;
        this.distance = distance;
        this.rotation = rotation;
        this.numFrames = numFrames;
    }

    @Override
    public boolean shouldTerminate(SensorData sensorData, HardwareData hardwareData) {
        if((position.getVector2().distanceTo(target.getVector2())) < distance && (Math.abs(MathUtils.getRadRotDist(position.getC(), Math.toRadians(target.getC()))) < Math.toRadians(rotation))) {
            frameCount++;
        }else{
            frameCount = 0;
        }
        return frameCount > numFrames;
    }
}
