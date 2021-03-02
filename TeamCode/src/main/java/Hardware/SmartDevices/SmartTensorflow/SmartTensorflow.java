package Hardware.SmartDevices.SmartTensorflow;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

import Hardware.CustomClasses.VuforiaPlus;
import Hardware.SmartDevices.SmartDevice;

public class SmartTensorflow extends SmartDevice {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "4";
    private static final String LABEL_SECOND_ELEMENT = "1";
    private static final String VUFORIA_KEY =
            "AWgMoiX/////AAABmWgxXRIPp0SKg/1vlWFXQHpvvUse8cgKFT6YVhd53nIsJoARWa0D3gSDe5KvRbJicI9jlwYY2wJ7ae7+Mx16NzOlGaOfoV5zRfNH8Ym3FCfjz9kXfjjgbpVfm6GARMvqHxfb7aTJAUf2+UFjjJhigeI16elpoZgAOpTYtXLmcT9Jj+P7ZZAz6rayYr11cfI5yj5QY2KDfiB0IPPH5FQpnixu6VyjN2EEMl4xGfDnPfCscG4wL66VX93BEh7vNyFyZE9+hmPSylugcYU5GeNRCE0JINYqZHvJ90ELPAlmUt4A+R6JfuzexKtwMU4PwBf2WrBxYmV+QT8208slL0CaGEZ34Re6qwPRLOyZ72LQ2+bI";
    private VuforiaPlus vuforia;
    private TFObjectDetector tfod;
    private int numDetected;
    private String labelDetected;
    private boolean active;
    private AtomicInteger rings;
    private Thread mainThread;

    public SmartTensorflow(WebcamName webcamName, HardwareMap hardwareMap){
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = webcamName;

        vuforia = new VuforiaPlus(parameters);

        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
        active = true;
    }


    @Override
    public void calibrate() {
        if(tfod != null){
            tfod.activate();
        }
    }

    @Override
    public void update() {

    }

    public int numDetected(){
        if (tfod != null && active) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                numDetected = updatedRecognitions.size();
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    labelDetected = recognition.getLabel();
                }
            }
        }
        if(numDetected > 0){
            if(labelDetected.equals(LABEL_FIRST_ELEMENT)){
                return 4;
            }else{
                return 1;
            }
        }else{
            return 0;
        }
    }

    public void deactivate(){
        active = false;
        tfod.deactivate();
        tfod.shutdown();
        vuforia.stop();
        tfod = null;
    }
}
