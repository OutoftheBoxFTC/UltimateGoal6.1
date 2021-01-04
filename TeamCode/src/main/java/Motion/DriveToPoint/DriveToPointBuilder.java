package Motion.DriveToPoint;
import MathUtils.*;
import State.StateMachine;

/**
 * Builds a standard drive to point class, returning a driveState that can be used in the statemachine
 */

public class DriveToPointBuilder {
    private StateMachine stateMachine;
    private double speed, rot;
    private Vector3 position;
    private Vector2 target;
    public DriveToPointBuilder(StateMachine stateMachine, Vector3 position){
        this.position = position;
        this.stateMachine = stateMachine;
        target = Vector2.ZERO();
        speed = 1;
        rot = 0;
    }

    public DriveToPointBuilder setSpeed(double speed){
        this.speed = speed;
        return this;
    }

    public DriveToPointBuilder setRot(double rot){
        this.rot = rot;
        return this;
    }

    public DriveToPointBuilder setTarget(Vector2 target){
        target.set(target);
        return this;
    }

    public DriveToPoint complete(){
        return new DriveToPoint(stateMachine, position, target.toVector3(rot), speed) {
            @Override
            public void setTarget() {
                localTarget.set(target.toVector3(rot));
            }
        };
    }
}
