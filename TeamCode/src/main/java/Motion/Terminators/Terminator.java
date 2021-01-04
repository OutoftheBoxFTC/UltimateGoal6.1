package Motion.Terminators;

import Hardware.Packets.*;

/**
 * Basic Terminator Class
 * The terminator is used to indicate the end of something, usually the end of a state or the end of motion
 */

public abstract class Terminator {
    public abstract boolean shouldTerminate(SensorData sensorData, HardwareData hardwareData);

    public static Terminator nullTerminator(){
        return new Terminator() {
            @Override
            public boolean shouldTerminate(SensorData sensorData, HardwareData hardwareData) {
                return false;
            }
        };
    }
}
