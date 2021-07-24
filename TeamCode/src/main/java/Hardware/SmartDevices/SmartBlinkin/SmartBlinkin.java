package Hardware.SmartDevices.SmartBlinkin;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.util.RobotLog;

import Hardware.SmartDevices.SmartDevice;

public class SmartBlinkin extends SmartDevice {
    private RevBlinkinLedDriver blinkin;
    private RevBlinkinLedDriver.BlinkinPattern pattern;
    private boolean changed = false;
    private long timer = 0;

    public SmartBlinkin(RevBlinkinLedDriver blinkin){
        this.blinkin = blinkin;
        this.pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
        blinkin.setPattern(pattern);
        timer = System.currentTimeMillis() + 1000;
    }

    @Override
    public void calibrate() {

    }

    @Override
    public void update() {
        if(changed && (System.currentTimeMillis() > timer)) {
            RobotLog.ii("Pattern Changed", pattern.toString());
            blinkin.setPattern(pattern);
            changed = false;
            //timer = System.currentTimeMillis() + 500;
        }
    }

    public RevBlinkinLedDriver.BlinkinPattern getPattern() {
        return pattern;
    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        if(!(pattern == null) && !(pattern == this.pattern) && (System.currentTimeMillis() > timer)) {
            this.pattern = pattern;
            changed = true;
        }
    }
}
