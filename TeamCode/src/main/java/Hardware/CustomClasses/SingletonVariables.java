package Hardware.CustomClasses;

import MathSystems.Vector3;

public class SingletonVariables {
    private static final SingletonVariables instance = new SingletonVariables();
    private Vector3 position;
    public SingletonVariables(){
        this.position = Vector3.ZERO();
    }

    public Vector3 getPosition() {
        return position;
    }

    public void setPosition(Vector3 position) {
        this.position.set(position);
    }

    public static SingletonVariables getInstance(){
        return instance;
    }
}
