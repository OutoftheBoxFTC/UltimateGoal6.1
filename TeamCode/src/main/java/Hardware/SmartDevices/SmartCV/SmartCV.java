package Hardware.SmartDevices.SmartCV;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

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

    private OpenCvWebcam tower;
    private WebcamName towerCam;
    private RingMaskPipeline ringPipeline;
    private TensorPipeline highgoalPipeline;
    int cameraMonitorViewId;
    private boolean opened;
    private boolean blue = false;

    public SmartCV(WebcamName towerCam, final HardwareMap hardwareMap){
        ringPipeline = new RingMaskPipeline();
        highgoalPipeline = new TensorPipeline(hardwareMap, 70, "model2.tflite");
        //highgoalPipeline = new TensorPipeline(hardwareMap, 70, "detectNew.tflite");
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        tower = OpenCvCameraFactory.getInstance().createWebcam(towerCam, cameraMonitorViewId);

        this.towerCam = towerCam;

        opened = false;
        //tower.setPipeline(highgoalPipeline);
        //FtcDashboard.getInstance().startCameraStream(tower, 30);
        /**tower.startRecordingPipeline(new PipelineRecordingParameters.Builder()
         .setPath("/sdcard/tower" + System.currentTimeMillis() + ".mp4")
         .setFrameRate(24)
         .setOutputFormat(PipelineRecordingParameters.OutputFormat.MPEG_4)
         .setEncoder(PipelineRecordingParameters.Encoder.H263)
         .setBitrate(3, PipelineRecordingParameters.BitrateUnits.Mbps)
         .build());
         */
    }

    @Override
    public void calibrate() {

    }

    @Override
    public void update() {

    }

    public void disableRingTrack(){

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

    public void setBlue(final boolean blue){

    }

    public OpenCvWebcam getTower() {
        return tower;
    }
}
