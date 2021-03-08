package Motion.Kinematics.NormalDriveToPoint;

import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import MathSystems.MathUtils;
import MathSystems.Vector3;
import State.StateMachine;
import State.VelocityDriveState;

public abstract class NormalDriveToPoint extends VelocityDriveState {
    public Vector3 position, localTarget;
    public Vector3 target, velocity;
    public double power = 0, rotPrec, powerMod;
    public NormalDriveToPoint(StateMachine stateMachine, Vector3 position, Vector3 target, double power) {
        super(stateMachine);
        this.position = position;
        this.target = target;
        localTarget = Vector3.ZERO();
        this.power = power;
        this.velocity = Vector3.ZERO();
        rotPrec = 1;
    }

    public NormalDriveToPoint(StateMachine stateMachine, Vector3 position, Vector3 target, double power, double rotPrec) {
        super(stateMachine);
        this.position = position;
        this.target = target;
        localTarget = Vector3.ZERO();
        this.power = power;
        this.velocity = Vector3.ZERO();
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
        double theta = Math.atan2(errY, errX) - position.getC();
        double errRot = MathUtils.getRadRotDist(position.getC(), Math.toRadians(localTarget.getC()));
        double comb = Math.abs(((r * Math.cos(theta))) + ((r * Math.sin(theta))));
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
            errRot = (errRot / Math.abs(errRot)) * (0.12 / power);
        }
        double x = (r * Math.cos(theta))/r;
        double y = (r * Math.sin(theta))/r;
        //RobotLog.ii("Test", x + ", " + y);
        velocity.set(x * power * powerMod, -y * power * powerMod, errRot * power * rotMod);
    }

    public abstract void setTarget();
}