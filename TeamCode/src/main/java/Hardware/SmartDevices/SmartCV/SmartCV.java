package Hardware.SmartDevices.SmartCV;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvSwitchableWebcam;
import org.openftc.easyopencv.PipelineRecordingParameters;

import Hardware.SmartDevices.SmartCV.TowerGoal.PipelineTester;
import Hardware.SmartDevices.SmartDevice;

public class SmartCV extends SmartDevice {

    private OpenCvCamera ring, tower;
    private WebcamName ringCam, towerCam;
    private RingPipeline ringPipeline;
    private PipelineTester highgoalPipeline;
    private boolean opened;

    public SmartCV(WebcamName ringCam, WebcamName towerCam, HardwareMap hardwareMap){
        ringPipeline = new RingPipeline();
        highgoalPipeline = new PipelineTester(hardwareMap.appContext, 60);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        ring = OpenCvCameraFactory.getInstance().createWebcam(ringCam);
        tower = OpenCvCameraFactory.getInstance().createWebcam(towerCam, cameraMonitorViewId);

        this.towerCam = towerCam;
        this.ringCam = ringCam;

        opened = false;

        ring.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                ring.openCameraDevice();
                ring.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                ring.setPipeline(ringPipeline);
            }
        });
        tower.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                tower.openCameraDevice();
                tower.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                tower.setPipeline(highgoalPipeline);
                /**tower.startRecordingPipeline(new PipelineRecordingParameters.Builder()
                .setPath("/sdcard/tower" + System.currentTimeMillis() + ".mp4")
                .setFrameRate(24)
                .setOutputFormat(PipelineRecordingParameters.OutputFormat.MPEG_4)
                .setEncoder(PipelineRecordingParameters.Encoder.H263)
                .setBitrate(3, PipelineRecordingParameters.BitrateUnits.Mbps)
                .build());
                 */
                //FtcDashboard.getInstance().startCameraStream(tower, 30);

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

    public int getRings(){
        return ringPipeline.getNumRings();
    }

    public double getArea(){
        return ringPipeline.getArea();
    }

    public double getHeading(){
        return highgoalPipeline.getHeading()-10;
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

    public double[] getPowershots(){
        double[] arr = highgoalPipeline.getPowershots();
        return new double[]{arr[0]-2, arr[1]-1, arr[2]-2};
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

    public void shutdown(){
        //openCvCamera.stopStreaming();
        //openCvCamera.closeCameraDevice();
    }
}
