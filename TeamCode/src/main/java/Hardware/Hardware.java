package Hardware;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.*;
import java.util.concurrent.atomic.AtomicBoolean;

import Hardware.Packets.*;
import Hardware.SmartDevices.*;

/**
 * Generic Hardware class
 *
 * The hardware class implements runnable, meaning it can be threaded for maximum performance
 */

public abstract class Hardware implements Runnable {
    public SmartDeviceMap smartDevices;
    public ArrayList<HardwareDevices> registeredDevices, enabledDevices;
    public LinearOpMode opMode;
    private final ArrayList<HardwareData> hardwarePackets;
    private final ArrayList<SensorData> sensorPackets;
    private AtomicBoolean end, available;
    private ArrayList<LynxModule> revHubs;

    public Hardware(){
        smartDevices = new SmartDeviceMap();
        hardwarePackets = new ArrayList<>();
        hardwarePackets.add(new HardwareData());
        sensorPackets = new ArrayList<>();
        sensorPackets.add(new SensorData());
        sensorPackets.add(new SensorData());
        registeredDevices = new ArrayList<>();
        enabledDevices = new ArrayList<>();
        end = new AtomicBoolean(false);
        available = new AtomicBoolean(false);
        revHubs = new ArrayList<>();
    }

    public void attachOpmode(LinearOpMode opMode){
        this.opMode = opMode;
    }

    /**
     * Registers devices, adding them to the SmartDevice ArrayList
     */

    public abstract void registerDevices(HardwareMap map);

    public void init(){
        revHubs.addAll(opMode.hardwareMap.getAll(LynxModule.class));
        for(LynxModule m : revHubs){
            m.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            m.clearBulkCache();
        }
        registerDevices(opMode.hardwareMap);
        for(String s : smartDevices.keySet()){
            smartDevices.get(s).setName(s);
            smartDevices.get(s).calibrate();
        }
    }

    /**
     * Reads in hardware data, using the data to update smart devices. Do not call update() from this method
     */

    public abstract void setHardware(HardwareData hardware);

    /**
     * Puts data from smart devices into the sensor data packet. Do not call update() from this method
     */

    public abstract void setSensors(SensorData sensorData);

    /**
     * Fetches the first sensor data packet from the array. An array is used to keep the Sensor Data threadsafe
     */

    public SensorData getSensorData(){
        while(!available.get());
        synchronized (sensorPackets) {
            SensorData out = sensorPackets.get(0);
            if (sensorPackets.size() > 1) {
                sensorPackets.remove(0);
            }
            available.set(false);
            return out;
        }
    }

    public SmartDeviceMap getSmartDevices(){
        return smartDevices;
    }

    /**
     * Adds a hardware packet to the array. An array is used to keep the Hardware Data threadsafe
     */

    public void addHardwarePacket(HardwareData hardware){
        synchronized (hardwarePackets) {
            if(hardwarePackets.size() > 0){
                hardwarePackets.clear();
            }
            hardwarePackets.add(hardware);
        }
    }

    @Override
    public void run(){
        while(!end.get()) {
            HardwareData hardwarePacket;
            synchronized (hardwarePackets) {
                hardwarePacket = hardwarePackets.get(0);
                if (hardwarePackets.size() > 1) {
                    hardwarePackets.remove(0);
                }
            }
            setHardware(hardwarePacket);
            for (String device : smartDevices.keySet()) {
                smartDevices.get(device).update();
            }
            SensorData sensorData = new SensorData();
            setSensors(sensorData);
            sensorData.setTimestamp(System.currentTimeMillis());
            synchronized (sensorPackets) {
                sensorData.setBacklog(hardwarePackets.size());
                if(sensorPackets.size() > 0){
                    sensorPackets.clear();
                }
                sensorPackets.add(sensorData);
            }
            available.set(true);
        }
        for(String s : smartDevices.keySet()){
            smartDevices.get(s).stop();
        }
    }

    public void enableDevice(HardwareDevices device){
        enabledDevices.add(device);
    }

    public void registerDevice(HardwareDevices device){
        registeredDevices.add(device);
    }

    public void registerAll(){
        registeredDevices.add(HardwareDevices.DRIVE_MOTORS);
        registeredDevices.add(HardwareDevices.INTAKE);
        registeredDevices.add(HardwareDevices.ODOMETRY);
        registeredDevices.add(HardwareDevices.GYRO);
        registeredDevices.add(HardwareDevices.SHOOTER);
        registeredDevices.add(HardwareDevices.WOBBLE);
        registeredDevices.add(HardwareDevices.WEBCAM);
        registeredDevices.add(HardwareDevices.TURRET);
    }

    public void enableAll(){
        enabledDevices.add(HardwareDevices.DRIVE_MOTORS);
        enabledDevices.add(HardwareDevices.INTAKE);
        enabledDevices.add(HardwareDevices.ODOMETRY);
        enabledDevices.add(HardwareDevices.GYRO);
        enabledDevices.add(HardwareDevices.SHOOTER);
        enabledDevices.add(HardwareDevices.WOBBLE);
        enabledDevices.add(HardwareDevices.WEBCAM);
        enabledDevices.add(HardwareDevices.TURRET);
    }

    public void disableAll(){
        enabledDevices.clear();
    }

    public void disableDevice(HardwareDevices device){
        enabledDevices.remove(device);
    }

    public void deRegisterDevice(HardwareDevices device){
        registeredDevices.remove(device);
    }

    public void stop(){
        end.set(true);
    }

    public static enum HardwareDevices {
        DRIVE_MOTORS,
        INTAKE,
        GYRO,
        ODOMETRY,
        SHOOTER,
        WOBBLE,
        WEBCAM,
        TURRET
    }
}
