package Hardware.Robots;

import MathSystems.PIDFSystem;
import MathSystems.PIDSystem;

public class RobotConstants {
    public class Skystone {
        public static final double ODOMETRY_PPR = 4096;
        public static final double ODOMETRY_TRANSLATION_FACTOR = 0.0011;
        public static final double ODOMETRY_TRACK_WIDTH = 7149.42;
    }
    public class UltimateGoal {
        public static final double ODOMETRY_TRANSLATION_FACTOR = (1/1849.206349206349); //1,816.810276679842
        public static final double ONEUSE_RIGHT_ARM_HOLD = 0.85; //0.29249
        public static final double ONEUSE_RIGHT_ARM_RELEASE = 0.29249; //0.85

        public static final double HOLD_INTAKE = 0.25;
        public static final double RELEASE_INTAKE = 0.85;
        public static final double IDLE_INTAKE = 0.4;

        public static final double MAX_ROTATION_SPEED = 2.5;
        public static final double MAX_SPEED = 30;

        public static final double MAX_LIN_ACCEL = 5;
        public static final double MAX_R_ACCEL = 2;

        public static final double KF = 0.15;
    }
}
