package Hardware.SmartDevices.SmartCV.TowerGoal;

import android.content.Context;
import android.graphics.Bitmap;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.teamcode.R;
import org.tensorflow.lite.DataType;
import org.tensorflow.lite.support.image.TensorImage;
import org.tensorflow.lite.task.vision.detector.Detection;
import org.tensorflow.lite.task.vision.detector.ObjectDetector;

import java.io.IOException;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;


public class TensorPipeline extends OpenCvPipeline {
    ObjectDetector det;
    Mat refMat;
    private Mat cropCopy, test;
    private MatOfPoint refContour;
    private Rect boundingRect;
    private double[] firingSolution = new double[]{0, 0}, powershots = new double[]{0, 0, 0}, position = new double[]{0, 0};
    private double fov, pitch, pitchOffset;
    private AtomicBoolean track = new AtomicBoolean(false);
    private int idx = 1;
    public TensorPipeline(HardwareMap hardwareMap, double fov){
        try {
            det = ObjectDetector.createFromFileAndOptions(hardwareMap.appContext, "model.tflite", ObjectDetector.ObjectDetectorOptions.builder().setNumThreads(2).setScoreThreshold(0.95f).build());
            try {
                refMat = Utils.loadResource(hardwareMap.appContext, R.raw.tower, Imgcodecs.IMREAD_GRAYSCALE);
            } catch (IOException e) {
                e.printStackTrace();
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
        this.fov = fov;
    }
    
    @Override
    public Mat processFrame(Mat input) {
        cropCopy = input.clone();
        Bitmap bmp = Bitmap.createBitmap(input.cols(), input.rows(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(input, bmp);
        List<Detection> dets = det.detect(TensorImage.createFrom(TensorImage.fromBitmap(bmp), DataType.UINT8));

        for(Detection d : dets) {
            Rect r = new Rect(
                    new Point(d.getBoundingBox().right, d.getBoundingBox().top),
                    new Point(d.getBoundingBox().left, d.getBoundingBox().bottom)
            );
            if(r.area() > 0) {
                double distance = 20;

                Imgproc.rectangle(input, r, new Scalar(0, 255, 0), 3);
                Imgproc.putText(input, "" + distance, CvUtils.getCenter(r), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0));

                double horDist = BetterTowerGoalUtils.approximateDistanceToGoal(23.87500, r.width, 817.063304531327);
                double verDist = BetterTowerGoalUtils.approximateDistanceToGoal(15.75, r.height, 819.4690054531818);

                firingSolution = PnPUtils.getPitchAndYaw(input, r, cropCopy, fov);

                pitch = firingSolution[0];

                double goalWallDist = (horDist + verDist) / 2;

                double goalWallDist2 = BetterTowerGoalUtils.getDistanceToGoalWall(14, firingSolution[0] + pitchOffset);
                powershots = BetterTowerGoalUtils.approxPowershotAngles(-firingSolution[1], goalWallDist2);

                firingSolution[0] = goalWallDist2;

                DecimalFormat format = new DecimalFormat("#.##");
                ////Imgproc.putText(cropCopy, "Firing Solution: ", new Point(2, input.height() - 170), 1, 2, new Scalar(0, 0, 0));
                //Imgproc.putText(cropCopy, "Pitch " + format.format(tmp[0]), new Point(2, input.height() - 100), 1, 3, new Scalar(0, 0, 0));
                ////Imgproc.putText(cropCopy, "Range " + format.format((goalWallDist2)), new Point(2, input.height() - 100), 1, 3, new Scalar(0, 0, 0));
                ////Imgproc.putText(cropCopy, "Heading " + format.format(firingSolution[1]), new Point(2, input.height() - 10), 1, 2.75, new Scalar(0, 0, 0));
                //Imgproc.putText(cropCopy, "Err " + format.format(horDist-verDist), new Point(2, input.height() - 10), 1, 3, new Scalar(0, 0, 0));

                double xDist = BetterTowerGoalUtils.approximateGoalX(goalWallDist2, -firingSolution[1]);
                position = new double[]{xDist, -goalWallDist2};
                track.set(true);
            }
        }
        return input;
    }

    public void setPitchOffset(double pitchOffset) {
        this.pitchOffset = pitchOffset;
    }

    public double getHeading(){
        return -firingSolution[1];
    }

    public double getPitch(){
        return pitch + pitchOffset;
    }

    public double getRange(){
        return firingSolution[0];
    }

    public boolean getTrack() {
        return track.get();
    }

    public double[] getPowershots() {
        return powershots;
    }

    public double calibratePitch(){
        return BetterTowerGoalUtils.approximateCameraAngle(14, 126, pitch);
    }

    public double[] getPosition() {
        return position;
    }

    public Rect getBoundingRect() {
        return boundingRect;
    }
}
