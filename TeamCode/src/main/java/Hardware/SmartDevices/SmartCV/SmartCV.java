package Hardware.SmartDevices.SmartCV;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;
import java.util.concurrent.TimeUnit;

import Hardware.SmartDevices.SmartCV.TowerGoal.ImageClassification.Classifier;
import Hardware.SmartDevices.SmartCV.TowerGoal.ImageClassification.TensorRingPipeline;
import Hardware.SmartDevices.SmartCV.TowerGoal.TensorPipeline;
import Hardware.SmartDevices.SmartDevice;
import MathSystems.Vector3;

public class SmartCV extends SmartDevice {

    private OpenCvWebcam ring, tower;
    private WebcamName ringCam, towerCam;
    private RingMaskPipeline ringPipeline;
    private TensorPipeline highgoalPipeline;
    private boolean opened;

    public SmartCV(WebcamName ringCam, WebcamName towerCam, final HardwareMap hardwareMap){
        ringPipeline = new RingMaskPipeline();
        highgoalPipeline = new TensorPipeline(hardwareMap, 70, "model2.tflite");
        //highgoalPipeline = new TensorPipeline(hardwareMap, 70, "detectNew.tflite");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        ring = OpenCvCameraFactory.getInstance().createWebcam(ringCam, cameraMonitorViewId);
        tower = OpenCvCameraFactory.getInstance().createWebcam(towerCam);

        this.towerCam = towerCam;
        this.ringCam = ringCam;

        opened = false;

        ring.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                ring.openCameraDevice();
                ring.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                ring.setPipeline(ringPipeline);
                FtcDashboard.getInstance().startCameraStream(ring, 30);
            }
        });
        tower.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                tower.openCameraDevice();
                tower.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
                tower.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
                tower.setPipeline(highgoalPipeline);
                /**tower.startRecordingPipeline(new PipelineRecordingParameters.Builder()
                .setPath("/sdcard/tower" + System.currentTimeMillis() + ".mp4")
                .setFrameRate(24)
                .setOutputFormat(PipelineRecordingParameters.OutputFormat.MPEG_4)
                .setEncoder(PipelineRecordingParameters.Encoder.H263)
                .setBitrate(3, PipelineRecordingParameters.BitrateUnits.Mbps)
                .build());
                 */

            }
        });
    }

    @Override
    public void calibrate() {

    }

    @Override
    public void update() {

    }

    public void disableRingTrack(){
        ring.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
            @Override
            public void onClose() {

            }
        });
    }

    public void shutdownTowerTrack(){
        highgoalPipeline.shutdownTensorflow();
    }

    public int getRings(){
        return ringPipeline.getNumRings();
    }

    //public double getArea(){
    //    return ringPipeline.getArea();
    //}

    //public List<Classifier.Recognition> getRecogs(){
    //    return ringPipeline.getMasterRecogs();
    //}

    public double getRedHeading(){
        return highgoalPipeline.getRedHeading()-0;
    }

    public double getBlueHeading(){
        return highgoalPipeline.getBlueHeading();
    }

    public double getRange(){
        return highgoalPipeline.getRange();
    }

    public boolean getTrack(){
        return highgoalPipeline.getTrack();
    }

    public double getPitch(){
        return highgoalPipeline.getPitch();
    }

    public double[] getRedPowershots(){
        double[] arr = highgoalPipeline.getRedPowershots();
        return new double[]{arr[0]-2, arr[1]-1, arr[2]-2};
    }

    public double[] getBluePowershots(){
        return highgoalPipeline.getBluePowershots();
    }

    public double calibratePitch(){
        double pitchOffset = highgoalPipeline.calibratePitch();
        highgoalPipeline.setPitchOffset(pitchOffset);
        return pitchOffset;
    }

    public double[] getPosition(){
        return highgoalPipeline.getPosition();
    }

    public void setPitchOffset(double pitchOffset){
        highgoalPipeline.setPitchOffset(pitchOffset);
    }

    public long getTimestamp(){
        return highgoalPipeline.getTimestamp();
    }

    public long getDataTimestamp(){
        return highgoalPipeline.getDataTimestamp();
    }

    public void setVelocity(Vector3 velocity){
        highgoalPipeline.setVelocity(velocity);
    }

    public void shutdown(){
        //openCvCamera.stopStreaming();
        //openCvCamera.closeCameraDevice();
    }

    public void setOuter(boolean outer){
        ringPipeline.setOuter(outer);
    }

    public void setBlue(boolean blue){
        ringPipeline.setBlue(blue);
    }
}
