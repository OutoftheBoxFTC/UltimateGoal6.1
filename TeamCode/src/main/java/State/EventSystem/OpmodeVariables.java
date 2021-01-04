package State.EventSystem;

import java.util.*;

import MathUtils.*;

/**
 * OpMode variables that can be passed as a group to classes
 * Classes such as LinearTrigger require the numbers to be put in here
 * This can be used to pass copies of primitive types that can be updated
 */

public class OpmodeVariables {
    public HashMap<String, Vector3> vector3;
    public HashMap<String, Vector2> vector2;
    public HashMap<String, Double> doubles;
    public HashMap<String, Integer> integers;
    public HashMap<String, String> string;
    public OpmodeVariables(){
        this.vector2 = new HashMap<>();
        this.vector3 = new HashMap<>();
        this.doubles = new HashMap<>();
        this.string = new HashMap<>();
        this.integers = new HashMap<>();
    }
}
