package Motion.Terminators;

import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import MathSystems.MathUtils;
import MathSystems.Vector3;
import Motion.Path.Path;

/**
 * Orientation Terminator
 * Creates a terminator that terminates when the position gets within a specified distance of the target
 */

public class OrientationTerminator extends Terminator {
    private Vector3 position, target;
    private double distance, rotation;
    private int frameCount = 0, numFrames;

    public OrientationTerminator(Vector3 position, Path path){
        this(position, path.getEndpoint().getVector2().toVector3(Math.toDegrees(path.getEndpoint().getC())), 1.5, 3);
    }

    public OrientationTerminator(Vector3 position, Path path, int numFrames){
        this(position, path.getEndpoint().getVector2().toVector3(Math.toDegrees(path.getEndpoint().getC())), 1.5, 3, numFrames);
    }

    public OrientationTerminator(Vector3 position, Vector3 target, double distance, double rotation){
        this.position = position;
        this.target = target;
        this.distance = distance;
        this.rotation = rotation;
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
