package Hardware.SmartDevices.Smart2MSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Smart2MSensorConfiguration {
    DistanceUnit unit;
    boolean relative;
    public Smart2MSensorConfiguration(){
        unit = DistanceUnit.INCH;
        relative = false;
    }
    public Smart2MSensorConfiguration setUnit(DistanceUnit unit){
        this.unit = unit;
        return this;
    }
    public Smart2MSensorConfiguration setRelativeDistance(boolean relativeDistance){
        this.relative = relativeDistance;
        return this;
    }
}
