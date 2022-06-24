package OpModes.Debug;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import Hardware.*;
import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import Hardware.SmartDevices.SmartCV.TowerGoal.TensorPipeline;
import OpModes.BasicOpmode;
import State.LogicState;

@TeleOp
public class DashboardTests extends BasicOpmode {
    public DashboardTests() {
        super(new TestHardware());
    }

    @Override
    public void setup() {
        eventSystem.onInit("Test", new LogicState(stateMachine) {
            TensorPipeline highgoalPipeline;
            @Override
            public void init(SensorData sensorData, HardwareData hardwareData) {
                final OpenCvCamera tower = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);
                int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
                highgoalPipeline = new TensorPipeline(hardwareMap, 70, "detectNew.tflite");
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
                FtcDashboard.getInstance().startCameraStream(tower, 30);
            }

            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {

            }
        });
    }
}
