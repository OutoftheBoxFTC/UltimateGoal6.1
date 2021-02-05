package Motion.Terminators;

import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;

public class TimeTerminator extends Terminator{
    double time = 1200;

    public TimeTerminator(double length){
        this.time = length;
    }

    @Override
    public boolean shouldTerminate(SensorData sensorData, HardwareData hardwareData) {
        time --;
        return time < 0;
    }
}
