package Motion.VelocityCorrectedDrive;

import MathSystems.Vector2;
import MathSystems.Vector3;
import State.StateMachine;

/**
 * Builds a standard drive to point class, returning a driveState that can be used in the statemachine
 */

public class VelocityDriveToPointBuilder {
    private StateMachine stateMachine;
    private double speed, rot, r1, r2, slowMod;
    private Vector3 position;
    private Vector2 target;
    public VelocityDriveToPointBuilder(StateMachine stateMachine, Vector3 position){
        this.position = position;
        this.stateMachine = stateMachine;
        target = Vector2.ZERO();
        speed = 1;
        rot = 0;
        r1 = 10;
        r2 = 0.25;
        slowMod = 25;
    }

    public VelocityDriveToPointBuilder setSpeed(double speed){
        this.speed = speed;
        return this;
    }

    public VelocityDriveToPointBuilder setRot(double rot){
        this.rot = rot;
        return this;
    }

    public VelocityDriveToPointBuilder setTarget(Vector2 target){
        this.target.set(target);
        return this;
    }

    public void setR1(double r1) {
        this.r1 = r1;
    }

    public void setR2(double r2) {
        this.r2 = r2;
    }

    public void setSlowMod(double slowMod) {
        this.slowMod = slowMod;
    }

    public VelocityDriveToPoint complete(){
        return new VelocityDriveToPoint(stateMachine, position, target.toVector3(rot), speed, r1, r2, slowMod) {
            @Override
            public void setTarget() {
                localTarget.set(target.toVector3(rot));
            }
        };
    }
}
