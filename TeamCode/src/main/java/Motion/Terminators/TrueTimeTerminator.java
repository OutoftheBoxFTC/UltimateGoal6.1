package Motion.Terminators;

import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import MathSystems.ProgramClock;

public class TrueTimeTerminator extends Terminator{
    double time = 1200;
    double ref = 0;

    public TrueTimeTerminator(double length){
        this.time = length;
        this.ref = length;
    }

    @Override
    public boolean shouldTerminate(SensorData sensorData, HardwareData hardwareData) {
        time -= ProgramClock.getFrameTimeMillis();
        return time < 0;
    }

    @Override
    public void reset() {
        time = ref;
    }
}
