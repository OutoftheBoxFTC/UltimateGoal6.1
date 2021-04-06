package Hardware.HarwareUtils;

import MathSystems.MathUtils;

public class UGUtils {
    public static double getTurretValue(double angle){
        angle = MathUtils.clamp(angle, -26, 20);
        return (((angle + 26.0) / 46.0) * 0.85) + 0.05;
    }

    public static boolean inRange(double angle){
        return (angle < 20) && (angle > -31);
    }
}
