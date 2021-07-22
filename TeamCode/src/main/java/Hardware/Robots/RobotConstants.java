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
        public static final double ODOMETRY_TRANSLATION_FACTOR = (1 / 1849.206349206349); //1,816.810276679842
        public static final double ONEUSE_RIGHT_ARM_HOLD = 0.85; //0.29249
        public static final double ONEUSE_RIGHT_ARM_RELEASE = 0.29249; //0.85

        public static final double ONEUSE_LEFT_ARM_HOLD = 0.2;
        public static final double ONEUSE_LEFT_ARM_RELEASE = 0.8;

        public static final double HOLD_INTAKE = 0.25;
        public static final double RELEASE_INTAKE = 0.85;
        public static final double IDLE_INTAKE = 0.4;

        public static final double MAX_ROTATION_SPEED = 3.75;
        public static final double MAX_SPEED = 30;

        public static final double MAX_LIN_ACCEL = 15;
        public static final double MAX_R_ACCEL = 1.5;

        public static final double KF = 0.1;
        public static final double rotFF = 0.1;

        public static final double INDEXER_IN_POSITION = 0;
        public static final double INDEXER_OUT_POSITION = 0;
        public static final double INDEXER_TIMING = 0;

        public static final double INTAKE_BLOCKER_DOWN = 1210;
        public static final double INTAKE_BLOCKER_UP = 2250;

        public static final double WOBBLE_ARM_LEFT_DOWN = 0.1;
        public static final double WOBBLE_ARM_RIGHT_DOWN = 0.9;

        public static final double WOBBLE_ARM_LEFT_CHANGE = 0.42;
        public static final double WOBBLE_ARM_RIGHT_CHANGE = 0.59;

        public static final double WOBBLE_ARM_LEFT_TRAVEL = 0.65;
        public static final double WOBBLE_ARM_RIGHT_TRAVEL = 0.35;

        public static final double WOBBLE_ARM_LEFT_SCORE = 0.65;
        public static final double WOBBLE_ARM_RIGHT_SCORE = 0.35;

        public static final double WOBBLE_FORK_LEFT_IN = 0.5208;
        public static final double WOBBLE_FORK_RIGHT_IN = 0.43622;

        public static final double WOBBLE_FORK_LEFT_OUT = 0.01;
        public static final double WOBBLE_FORK_RIGHT_OUT = 0.96634;

        public static final double WOBBLE_FORK_LEFT_TRAVEL = 0.18;
        public static final double WOBBLE_FORK_RIGHT_TRAVEL = 0.79634;
    }
}
