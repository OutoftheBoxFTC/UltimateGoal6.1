package Motion.DriveToPoint;

import com.qualcomm.robotcore.util.RobotLog;

import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import State.StateMachine;
import State.VelocityDriveState;
import MathSystems.*;

/**
 * Drives to a given point using a straight line method
 * the setTarget() function sets the target to drive to
 */

public abstract class DriveToPoint extends VelocityDriveState {
    public Vector3 position, localTarget;
    private Vector3 target, velocity;
    public double power = 0, r1, r2, slowMod, minimums, rotPrec;
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
        this.minimums = 0.125;
        rotPrec = 0.5;
    }

    public DriveToPoint(StateMachine stateMachine, Vector3 position, Vector3 target, double power, double r1, double r2, double slowMod, double minimums, double rotPrec) {
        super(stateMachine);
        this.position = position;
        this.target = target;
        localTarget = Vector3.ZERO();
        this.power = power;
        this.velocity = Vector3.ZERO();
        this.r1 = r1;
        this.r2 = r2;
        this.slowMod = slowMod;
        this.minimums = minimums;
        this.rotPrec = rotPrec;
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
        double comb = Math.abs(((r * Math.cos(theta))) + ((r * Math.sin(theta))));
        double powerMod = 1;
        if(r < r1){
            powerMod = r/slowMod;
            if((powerMod * power) < minimums){
                powerMod = minimums/power;
            }
        }
        if(r < r2){
            powerMod = 0;
        }
        double rotMod = 1;
        if(Math.abs(Math.toDegrees(MathUtils.getRadRotDist(position.getC(), Math.toRadians(localTarget.getC())))) < rotPrec){
            rotMod = 0;
        }
        if(Math.abs(errRot) < Math.toRadians(15) && errRot != 0){
            errRot = errRot * 0.25;
        }else if(errRot != 0){
            errRot = (Math.abs(errRot)/errRot) * 1;
        }
        if(Math.abs(errRot * power) < 0.12){
            if(powerMod != 0) {
                errRot = (errRot / Math.abs(errRot)) * (0.12 / power);
            }else{
                errRot = (errRot / Math.abs(errRot)) * (0.15 / power);
            }
        }
        double x = (r * Math.cos(theta))/r;
        double y = (r * Math.sin(theta))/r;
        RobotLog.ii("Test", x + ", " + y);
        velocity.set(x * power * powerMod, -y * power * powerMod, errRot * power * rotMod);
    }

    public abstract void setTarget();
}
