package Hardware.SmartDevices.SmartEncoder;

/**
 * Configuration class for a Smart Encoder
 * The direction variable is true when reversed, and false when normal
 */

public class SmartEncoderConfiguration {
    boolean direction;
    public SmartEncoderConfiguration(){
        direction = false;
    }

    public SmartEncoderConfiguration reverseDirection(){
        direction = true;
        return this;
    }
}
