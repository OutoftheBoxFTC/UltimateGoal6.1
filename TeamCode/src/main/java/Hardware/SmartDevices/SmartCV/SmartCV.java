package Hardware.SmartDevices.SmartCV;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import Hardware.SmartDevices.SmartDevice;

public class SmartCV extends SmartDevice {

    private OpenCvCamera openCvCamera;
    private RingPipeline ringPipeline;

    public SmartCV(WebcamName webcamName, HardwareMap hardwareMap){
        ringPipeline = new RingPipeline();
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        openCvCamera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        openCvCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                openCvCamera.startStreaming(1280, 720);
                openCvCamera.setPipeline(ringPipeline);
            }
        });
    }

    @Override
    public void calibrate() {

    }

    @Override
    public void update() {

    }

    public int getRings(){
        return ringPipeline.getNumRings();
    }

    public double getArea(){
        return ringPipeline.getArea();
    }

    public void shutdown(){
        //openCvCamera.stopStreaming();
        //openCvCamera.closeCameraDevice();
    }
}
