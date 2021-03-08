package Motion.Kinematics.PIDDriveToPoint;

import MathSystems.Vector2;
import MathSystems.Vector3;
import MathSystems.Vector4;
import State.StateMachine;

/**
 * Builds a standard drive to point class, returning a driveState that can be used in the statemachine
 */

public class PIDDriveToPointBuilder {
    private StateMachine stateMachine;
    private double speed, rot, slowdownDistance, rotationSlowdown, minimums;
    private Vector3 position, velocity;
    private Vector2 target;
    private Vector4 driveGain, rotGain;
    public PIDDriveToPointBuilder(StateMachine stateMachine, Vector3 position, Vector3 velocity){
        this.position = position;
        this.stateMachine = stateMachine;
        target = Vector2.ZERO();
        speed = 1;
        rot = 0;
        slowdownDistance = 5;
        rotationSlowdown = 10;
        this.velocity = velocity;
        this.minimums = 0.15;
        driveGain = new Vector4(0.02, 0.03, 0, 0);
        rotGain = new Vector4(3, 0.1, 0, 0);
    }

    public PIDDriveToPointBuilder setSpeed(double speed){
        this.speed = speed;
        return this;
    }

    public PIDDriveToPointBuilder setRot(double rot){
        this.rot = rot;
        return this;
    }

    public PIDDriveToPointBuilder setTarget(Vector2 target){
        this.target.set(target);
        return this;
    }

    public PIDDriveToPointBuilder setMinimums(double minimums){
        this.minimums = minimums;
        return this;
    }

    public PIDDriveToPointBuilder setSlowdownDistance(double slowdownDistance){
        this.slowdownDistance = slowdownDistance;
        return this;
    }

    public PIDDriveToPointBuilder setRotationSlowdown(double rotationSlowdown){
        this.rotationSlowdown = rotationSlowdown;
        return this;
    }

    public PIDDriveToPointBuilder setDriveGain(Vector4 driveGain){
        this.driveGain = driveGain;
        return this;
    }

    public PIDDriveToPointBuilder setRotGain(Vector4 rotGain){
        this.rotGain = rotGain;
        return this;
    }

    public PIDDriveToPoint complete(){
        return new PIDDriveToPoint(stateMachine, position, velocity, target.toVector3(rot), speed, slowdownDistance, rotationSlowdown, minimums, driveGain, rotGain) {
            @Override
            public void setTarget() {
                localTarget.set(target.toVector3(rot));
            }
        };
    }
}
