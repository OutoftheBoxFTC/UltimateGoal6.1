package Hardware.RobotSystems;

import MathUtils.*;

public class MecanumSystem {
    /**
     * Translates x y r to motor powers, where x is the strafe velocity, y is the forward velocity, and r is the rotational velocity
     * All velocities should be in the range (-1, 1)
     * @param coords a vector3 containing {x, y, r}
     * @return motor powers in a vector4
     */
    public static Vector4 translate(Vector3 coords){
        return new Vector4(-coords.getB() + coords.getA() - coords.getC(), coords.getB() + coords.getA() - coords.getC(), -coords.getB() - coords.getA() - coords.getC(), coords.getB() - coords.getA() - coords.getC());
    }
}
