package Motion.DriveToPoint;

import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import State.FieldCentricDriveState;
import State.StateMachine;
import State.VelocityDriveState;
import MathUtils.*;

/**
 * Drives to a given point using a straight line method
 * the setTarget() function sets the target to drive to
 */

public abstract class DriveToPoint extends VelocityDriveState {
    public Vector3 position, localTarget;
    private Vector3 target, velocity;
    double power = 0;
    public DriveToPoint(StateMachine stateMachine, Vector3 position, Vector3 target, double power) {
        super(stateMachine);
        this.position = position;
        this.target = target;
        localTarget = Vector3.ZERO();
        this.power = power;
        this.velocity = Vector3.ZERO();
    }

    @Override
    public Vector3 getVelocities() {
        return velocity;
    }

    @Override
    public void update(SensorData sensors, HardwareData hardware) {
        setTarget();
        double errY = localTarget.getB() - position.getA();
        double errX = localTarget.getB() - position.getB();
        double r = Math.sqrt((errX * errX) + (errY * errY));
        double theta = Math.atan2(errY, errX) + position.getC();
        double errRot = MathUtils.getRadRotDist(Math.toDegrees(position.getC()), localTarget.getC());
        errRot = Range.clip(errRot/15, -1, 1);
        if(Math.abs(errRot) < 0.2 && errRot != 0){
            errRot = (Math.abs(errRot)/errRot) * 0.2;
        }
        double comb = ((r * Math.cos(theta))) + ((r * Math.sin(theta)));
        double x = (r * Math.cos(theta))/comb;
        double y = (r * Math.sin(theta))/comb;
        velocity.set(x * power, -y * power, errRot * power);
    }

    public abstract void setTarget();
}
