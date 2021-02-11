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
    double power = 0, r1, r2, slowMod;
    public DriveToPoint(StateMachine stateMachine, Vector3 position, Vector3 target, double power, double r1, double r2, double slowMod) {
        super(stateMachine);
        this.position = position;
        this.target = target;
        localTarget = Vector3.ZERO();
        this.power = power;
        this.velocity = Vector3.ZERO();
        this.r1 = r1;
        this.r2 = r2;
        this.slowMod = slowMod;
    }

    @Override
    public Vector3 getVelocities() {
        return velocity;
    }

    @Override
    public void update(SensorData sensors, HardwareData hardware) {
        setTarget();
        double errY = localTarget.getB() - position.getB();
        double errX = localTarget.getA() - position.getA();
        double r = Math.sqrt((errX * errX) + (errY * errY));
        double theta = Math.atan2(errY, errX) + position.getC();
        double errRot = MathUtils.getRadRotDist(position.getC(), Math.toRadians(localTarget.getC()));
        if(Math.abs(errRot) < Math.toRadians(15) && errRot != 0){
            errRot = (Math.abs(errRot)/errRot) * 0.175;
        }else if(errRot != 0){
            errRot = (Math.abs(errRot)/errRot) * 1;
        }
        double comb = Math.abs(((r * Math.cos(theta))) + ((r * Math.sin(theta))));
        double powerMod = 1;
        if(r < r1){
            powerMod = r/slowMod;
            if((powerMod * power) < 0.2){
                powerMod = 0.2/power;
            }
        }
        if(r < r2){
            powerMod = 0;
        }
        double rotMod = 1;
        if(Math.abs(Math.toDegrees(MathUtils.getRadRotDist(position.getC(), Math.toRadians(localTarget.getC())))) < 1.5){
            rotMod = 0;
        }
        double x = (r * Math.cos(theta))/r;
        double y = (r * Math.sin(theta))/r;
        RobotLog.ii("Test", x + ", " + y);
        velocity.set(x * power * powerMod, -y * power * powerMod, errRot * power * rotMod);
    }

    public abstract void setTarget();
}
