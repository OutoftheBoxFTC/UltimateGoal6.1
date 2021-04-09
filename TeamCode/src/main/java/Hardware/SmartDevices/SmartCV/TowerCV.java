package Hardware.SmartDevices.SmartCV;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import Hardware.SmartDevices.SmartCV.TowerGoal.PipelineTester;
import Hardware.SmartDevices.SmartDevice;

public class TowerCV extends SmartDevice {

    private OpenCvCamera openCvCamera;
    private PipelineTester ringPipeline;

    public TowerCV(WebcamName webcamName, HardwareMap hardwareMap){
        ringPipeline = new PipelineTester(hardwareMap.appContext, 60);
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        openCvCamera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        openCvCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                openCvCamera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
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

    public double getHeading(){
        return ringPipeline.getHeading()-10;
    }

    public double getRange(){
        return ringPipeline.getRange();
    }

    public boolean getTrack(){
        return ringPipeline.getTrack();
    }

    public double[] getPowershots(){
        double[] arr = ringPipeline.getPowershots();
        return new double[]{arr[0]-12, arr[1]-11, arr[2]-10};
    }

    public void shutdown(){
        //openCvCamera.stopStreaming();
        //openCvCamera.closeCameraDevice();
    }
}
