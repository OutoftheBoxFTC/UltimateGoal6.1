package Hardware.HarwareUtils;

import com.acmerobotics.dashboard.config.Config;

import MathSystems.MathUtils;
@Config
public class UGUtils {
    public static double minAngle = -35, maxAngle = 28.5;
    public static double getTurretValue(double angle){
        angle = MathUtils.clamp(angle, minAngle, maxAngle);
        return (((angle + Math.abs(minAngle)) / (maxAngle + Math.abs(minAngle))) * 0.9) + 0.05;
    }

    public static boolean inRange(double angle){
        return (angle <= maxAngle) && (angle >= minAngle);
    }

    public static double PWM_TO_SERVO(double pwm){
        return (pwm - 500) / (2000);
    }
}
