package Hardware.SmartDevices.SmartCV.TowerGoal;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.checkerframework.checker.units.qual.A;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.tensorflow.lite.DataType;
import org.tensorflow.lite.support.image.TensorImage;
import org.tensorflow.lite.task.core.vision.ImageProcessingOptions;
import org.tensorflow.lite.task.vision.detector.Detection;
import org.tensorflow.lite.task.vision.detector.ObjectDetector;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;

import Hardware.SmartDevices.SmartCV.TowerGoal.TFLite.Detector;
import Hardware.SmartDevices.SmartCV.TowerGoal.TFLite.TFLiteObjectDetectionAPIModel;
import MathSystems.Vector3;


public class TensorPipeline extends OpenCvPipeline {
    ObjectDetector det;
    private Rect boundingRect;
    private double[] redFiringSolution = new double[]{0, 0}, redPowershots = new double[]{0, 0, 0}, blueFiringSolution = new double[]{0, 0}, bluePowershots = new double[]{0, 0, 0}, position = new double[]{0, 0};
    private AtomicLong timestamp = new AtomicLong(0), dataTimestamp = new AtomicLong(0);
    private final double fov;
    private double pitch;
    private double pitchOffset;
    private final AtomicBoolean track = new AtomicBoolean(false);
    private Vector3 velocity = Vector3.ZERO();
    private AtomicBoolean shutdown = new AtomicBoolean(false);
    Detector detector;


    public TensorPipeline(HardwareMap hardwareMap, double fov, String model){
        try {
            det = ObjectDetector.createFromFileAndOptions(hardwareMap.appContext, model, ObjectDetector.ObjectDetectorOptions.builder().setNumThreads(2).setScoreThreshold(0.95f).build());
            detector = TFLiteObjectDetectionAPIModel.create(hardwareMap.appContext, "model (3).tflite", 320, true, 25, "Goal", "Null");
            detector.setUseNNAPI(true);
        } catch (IOException e) {
            e.printStackTrace();
        }
        this.fov = fov;
    }

    @Override
    public Mat processFrame(Mat input) {
        if(Math.toDegrees(Math.abs(velocity.getC())) > 5 || shutdown.get()){
            return input;
        }
        timestamp.set(System.currentTimeMillis());
        Mat cropCopy = input.clone();
        Mat resized = new Mat();
        Imgproc.resize(input, resized, new Size(320, 320));
        Bitmap bmp = Bitmap.createBitmap(resized.cols(), resized.rows(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(resized, bmp);

        //List<Detection> dets = det.detect(TensorImage.createFrom(TensorImage.fromBitmap(bmp), DataType.UINT8));
        List<Detector.Recognition> dets = detector.recognizeImage(bmp);
        //List<Detection> dets = new ArrayList<>();
        bmp.recycle();
        for(Detector.Recognition d : dets) {
            if(d.getConfidence() < 0.98f){
                continue;
            }
            Rect r = new Rect(
                    new Point((d.getLocation().right / 320) * input.width(), (d.getLocation().top / 320) * input.height()),
                    new Point((d.getLocation().left / 320) * input.width(), (d.getLocation().bottom / 320) * input.height())
            );
            boundingRect = r;
            track.set(false);
            if(r.area() > 0) {

                Scalar means = Core.mean(input.submat(CvUtils.cleanRect(r, input)));
                double distance = 20;

                Imgproc.rectangle(input, r, new Scalar(0, 255, 0), 2);

                if(means.val[0] > means.val[2]){
                    Imgproc.putText(input, "Red " + d.getConfidence(), CvUtils.getCenter(r), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0));
                    //The goal is the red goal (R > B)
                    redFiringSolution = PnPUtils.getPitchAndYaw(input, r, cropCopy, fov);
                    pitch = redFiringSolution[0];
                    double goalWallDist2 = BetterTowerGoalUtils.getDistanceToGoalWall(17.5, redFiringSolution[0] + pitchOffset);
                    redPowershots = BetterTowerGoalUtils.approxPowershotAngles(-redFiringSolution[1], goalWallDist2, BetterTowerGoalUtils.RED);
                    redFiringSolution[0] = goalWallDist2;
                    double xDist = BetterTowerGoalUtils.approximateGoalX(goalWallDist2, -redFiringSolution[1]);
                    position = new double[]{xDist + 35 + 6, -goalWallDist2}; //Subtract 35 inches so that the point 0, 0 is in the centre of the field
                }else{
                    Imgproc.putText(input, "Blue " + d.getConfidence(), CvUtils.getCenter(r), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0));
                    //The goal is the blue goal (R < B)
                    blueFiringSolution = PnPUtils.getPitchAndYaw(input, r, cropCopy, fov);
                    pitch = blueFiringSolution[0];
                    double goalWallDist2 = BetterTowerGoalUtils.getDistanceToGoalWall(17.5, blueFiringSolution[0] + pitchOffset);
                    bluePowershots = BetterTowerGoalUtils.approxPowershotAngles(-blueFiringSolution[1], goalWallDist2, BetterTowerGoalUtils.BLUE);
                    blueFiringSolution[0] = goalWallDist2;
                    double xDist = BetterTowerGoalUtils.approximateGoalX(goalWallDist2, -blueFiringSolution[1]);
                    position = new double[]{xDist - 35 + 6, -goalWallDist2}; //Add 35 inches so that the point 0, 0 is in the centre of the field
                }
                dataTimestamp.set(System.currentTimeMillis());
                track.set(true);
            }
        }
        cropCopy.release();
        return input;
    }

    public void setPitchOffset(double pitchOffset) {
        this.pitchOffset = pitchOffset;
    }

    public double getRedHeading(){
        return -redFiringSolution[1];
    }

    public double getBlueHeading(){
        return -blueFiringSolution[1];
    }

    public double getPitch(){
        return pitch + pitchOffset;
    }

    public double getRange(){
        return redFiringSolution[0];
    }

    public boolean getTrack() {
        return track.get();
    }

    public double[] getRedPowershots() {
        return redPowershots;
    }

    public double[] getBluePowershots() {
        return bluePowershots;
    }

    public long getTimestamp() {
        return timestamp.get();
    }

    public long getDataTimestamp() {
        return dataTimestamp.get();
    }

    public double calibratePitch(){
        return BetterTowerGoalUtils.approximateCameraAngle(17.5, 126, pitch);
    }

    public double[] getPosition() {
        return position;
    }

    public void setVelocity(Vector3 velocity) {
        this.velocity = velocity;
    }

    public void shutdownTensorflow(){
        shutdown.set(true);
    }

    public Rect getBoundingRect() {
        return boundingRect;
    }
}
