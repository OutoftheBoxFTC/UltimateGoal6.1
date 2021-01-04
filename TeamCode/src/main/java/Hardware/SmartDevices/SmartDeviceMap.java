package Hardware.SmartDevices;

import com.qualcomm.robotcore.util.RobotLog;

import java.util.*;

public class SmartDeviceMap {
    private HashMap<String, SmartDevice> deviceMap;

    public SmartDeviceMap(){
        deviceMap = new HashMap<>();
    }

    public void put(String name, SmartDevice device){
        deviceMap.put(name, device);
    }

    public Set<String> keySet(){
        return deviceMap.keySet();
    }

    public boolean containsKey(String key){
        return deviceMap.containsKey(key);
    }

    public HashMap<String, SmartDevice> getMap(){
        return deviceMap;
    }

    public SmartDevice get(String name){
        return deviceMap.get(name);
    }

    public <T> T get(String name, Class<? extends T> type){
        if(type.isInstance(deviceMap.get(name))){
            return type.cast(deviceMap.get(name));
        }
        RobotLog.e("Unable to find a Smart Device named " + name + " of type " + type.toString());
        return null;
    }
}
