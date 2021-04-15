package Hardware.HarwareUtils;

import com.acmerobotics.dashboard.config.Config;

import MathSystems.MathUtils;
@Config
public class UGUtils {
    public static double minAngle = -22, maxAngle = 27;
    public static double getTurretValue(double angle){
        angle = MathUtils.clamp(angle, minAngle, maxAngle);
        return (((angle + Math.abs(minAngle)) / (maxAngle + Math.abs(minAngle))) * 0.95) + 0.05;
    }

    public static boolean inRange(double angle){
        return (angle < maxAngle) && (angle > minAngle);
    }
}
