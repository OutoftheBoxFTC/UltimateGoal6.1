package Hardware.CustomClasses;

import MathSystems.Vector3;

public class SingletonVariables {
    private static final SingletonVariables instance = new SingletonVariables();
    private String prevState = "";
    private Vector3 position;
    private double pitchOffset = 17;
    public SingletonVariables(){
        this.position = Vector3.ZERO();
    }

    public Vector3 getPosition() {
        return position;
    }

    public void setPosition(Vector3 position) {
        this.position.set(position);
    }

    public void setPitchOffset(double pitchOffset) {
        this.pitchOffset = pitchOffset;
    }

    public double getPitchOffset() {
        return pitchOffset;
    }

    public String getPrevState() {
        return prevState;
    }

    public void setPrevState(String prevState) {
        this.prevState = prevState;
    }

    public static SingletonVariables getInstance(){
        return instance;
    }
}
