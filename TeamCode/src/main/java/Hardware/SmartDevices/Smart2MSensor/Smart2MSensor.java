package Hardware.SmartDevices.Smart2MSensor;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import Hardware.SmartDevices.SmartDevice;

public class Smart2MSensor extends SmartDevice {
    private Rev2mDistanceSensor sensor;
    private Smart2MSensorConfiguration configuration;
    private double distance, offset;
    public Smart2MSensor(Rev2mDistanceSensor sensor, Smart2MSensorConfiguration configuration){
        this.sensor = sensor;
        this.configuration = configuration;
        offset = 0;
        distance = 0;
    }

    @Override
    public void calibrate() {
        if(configuration.relative){
            offset = sensor.getDistance(configuration.unit);
        }
    }

    @Override
    public void update() {
        if(configuration.relative){
            distance = sensor.getDistance(configuration.unit) - offset;
        }else{
            distance = sensor.getDistance(configuration.unit);
        }
    }

    public double getDistance(){
        return distance;
    }

    @Override
    public String toString() {
        return "Smart2MSensor{" +
                "sensor=" + sensor +
                ", configuration=" + configuration +
                ", distance=" + distance +
                ", offset=" + offset +
                '}';
    }
}
