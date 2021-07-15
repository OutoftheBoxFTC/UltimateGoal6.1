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

    public TensorPipeline(HardwareMap hardwareMap, double fov, String model){
        try {
            det = ObjectDetector.createFromFileAndOptions(hardwareMap.appContext, model, ObjectDetector.ObjectDetectorOptions.builder().setNumThreads(4).setScoreThreshold(0.95f).build());
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
        Bitmap bmp = Bitmap.createBitmap(input.cols(), input.rows(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(input, bmp);

        List<Detection> dets = det.detect(TensorImage.createFrom(TensorImage.fromBitmap(bmp), DataType.UINT8));
        //List<Detection> dets = new ArrayList<>();
        bmp.recycle();
        for(Detection d : dets) {
            Rect r = new Rect(
                    new Point(d.getBoundingBox().right, d.getBoundingBox().top),
                    new Point(d.getBoundingBox().left, d.getBoundingBox().bottom)
            );
            boundingRect = r;
            track.set(false);
            if(r.area() > 0) {

                Scalar means = Core.mean(input.submat(CvUtils.cleanRect(r, input)));
                double distance = 20;

                Imgproc.rectangle(input, r, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, new Point(d.getBoundingBox().right, d.getBoundingBox().top), 5, new Scalar(255, 0, 0), -1);
                Imgproc.circle(input, new Point(d.getBoundingBox().right, d.getBoundingBox().bottom), 5, new Scalar(0, 255, 0), -1);
                Imgproc.circle(input, new Point(d.getBoundingBox().left, d.getBoundingBox().top), 5, new Scalar(0, 0, 255), -1);
                Imgproc.circle(input, new Point(d.getBoundingBox().left, d.getBoundingBox().bottom), 5, new Scalar(255, 255, 0), -1);

                if(means.val[0] > means.val[2]){
                    Imgproc.putText(input, "Red " + d.getCategories().get(0).getScore(), CvUtils.getCenter(r), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0));
                    //The goal is the red goal (R > B)
                    redFiringSolution = PnPUtils.getPitchAndYaw(input, r, cropCopy, fov);
                    pitch = redFiringSolution[0];
                    double goalWallDist2 = BetterTowerGoalUtils.getDistanceToGoalWall(17.5, redFiringSolution[0] + pitchOffset);
                    redPowershots = BetterTowerGoalUtils.approxPowershotAngles(-redFiringSolution[1], goalWallDist2, BetterTowerGoalUtils.RED);
                    redFiringSolution[0] = goalWallDist2;
                    double xDist = BetterTowerGoalUtils.approximateGoalX(goalWallDist2, -redFiringSolution[1]);
                    position = new double[]{xDist + 35 + 6, -goalWallDist2}; //Subtract 35 inches so that the point 0, 0 is in the centre of the field
                }else{
                    Imgproc.putText(input, "Blue " + d.getCategories().get(0).getScore(), CvUtils.getCenter(r), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0));
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
