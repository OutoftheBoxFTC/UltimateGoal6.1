package Hardware.SmartDevices.SmartCamera;

import java.util.concurrent.TimeUnit;

public class SmartCameraConfiguration {
    public boolean switchable;
    public int registerTimeout;
    public TimeUnit timeoutUnit;
    public SmartCameraConfiguration(){
        switchable = false;
        registerTimeout = 2;
        timeoutUnit = TimeUnit.SECONDS;
    }

    public SmartCameraConfiguration setSwitchable(){
        this.switchable = true;
        return this;
    }

    public SmartCameraConfiguration setTimeout(int timeout){
        this.registerTimeout = timeout;
        return this;
    }

    public SmartCameraConfiguration setTimeUnit(TimeUnit timeUnit){
        this.timeoutUnit = timeUnit;
        return this;
    }

    @Override
    public String toString() {
        return "SmartCameraConfiguration{" +
                "switchable=" + switchable +
                ", registerTimeout=" + registerTimeout +
                ", timeoutUnit=" + timeoutUnit +
                '}';
    }
}
