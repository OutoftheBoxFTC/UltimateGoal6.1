package Hardware.SmartDevices;

/**
 * Generic Smart Device class
 * Use Smart Devices instead of regular sensor classes
 */

public abstract class SmartDevice {
    String name;

    public SmartDevice(){
        this.name = "Unnamed Device";
    }

    public void setName(String name) {
        this.name = name;
    }

    public String getName() {
        return name;
    }

    /**
     * Calibrate method should be used to perform any initialization or calibration of the device
     */

    public abstract void calibrate();

    /**
     * The update method should perform any updating methods of the device
     * If the device is a hardware device, it will set the position or power of the device
     * If the device is a sensor, it should read data in
     *
     * This method must be thread safe!
     */

    public abstract void update();

    public void stop(){

    }
}
