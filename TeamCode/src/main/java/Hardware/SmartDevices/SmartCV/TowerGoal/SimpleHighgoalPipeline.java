package Hardware.SmartDevices.SmartCV.TowerGoal;

import android.content.Context;

import org.firstinspires.ftc.teamcode.R;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.IOException;
import java.text.DecimalFormat;
import java.util.concurrent.atomic.AtomicBoolean;

public class SimpleHighgoalPipeline extends OpenCvPipeline {
    private Mat cropCopy, test;
    private MatOfPoint refContour;
    private Rect boundingRect;
    private double[] firingSolution = new double[]{0, 0}, powershots = new double[]{0, 0, 0}, position = new double[]{0, 0};
    private double fov, pitch, pitchOffset;
    private AtomicBoolean track = new AtomicBoolean(false);
    private int idx = 1;

    public SimpleHighgoalPipeline(Context context, double fov){
        boundingRect = new Rect(0, 0, 0, 0);
        cropCopy = new Mat();
        test = new Mat();
        Mat refMat = new Mat();
        try {
            refMat = Utils.loadResource(context, R.raw.ref);
        } catch (IOException e) {
            e.printStackTrace();
        }

        this.fov = fov;
        this.pitchOffset = 17.5893;

        refContour = BetterTowerGoalUtils.getReference(refMat);
        refMat.release();
    }
    @Override
    public Mat processFrame(Mat input) {

        Mat resized = input.clone();
        Imgproc.cvtColor(resized, resized, Imgproc.COLOR_RGB2BGR);
        //resized = input.clone();
        //MatOfPoint m = BetterTowerGoalUtils.getMatchRect(refContour, resized);
        Mat cropped = BetterTowerGoalUtils.cropInRange(resized);
        MatOfPoint m = BetterTowerGoalUtils.getContour(cropped);

        input.copyTo(cropCopy);
        boundingRect = Imgproc.boundingRect(m);
        m = BetterTowerGoalUtils.refineEstimation(resized, boundingRect, cropCopy);

        MatOfPoint scl = new MatOfPoint(), scl2 = new MatOfPoint();
        for(Point p : m.toArray()){
            scl.push_back(new MatOfPoint(new Point((p.x/640) * input.width(), (p.y/480) * input.height())));
            scl2.push_back(new MatOfPoint(new Point((p.x/640) * 1280, (p.y/480) * 720)));
        }

        boundingRect = Imgproc.boundingRect(scl);

        Imgproc.rectangle(cropCopy, boundingRect, new Scalar(0, 255, 0), 5);

        if(boundingRect.area() > 5) {
            //CvUtils.drawRRect(cropCopy, Imgproc.minAreaRect(new MatOfPoint2f(scl.toArray())), new Scalar(0, 255, 0), 5);

            Imgproc.rectangle(cropCopy, new Point(0, input.height() - 200), new Point(350, input.height()), new Scalar(255, 255, 255), -1);

            //817.063304531327
            double horDist = BetterTowerGoalUtils.approximateDistanceToGoal(23.87500, Imgproc.boundingRect(scl2).width, 817.063304531327);
            double verDist = BetterTowerGoalUtils.approximateDistanceToGoal(15.75, Imgproc.boundingRect(scl2).height, 819.4690054531818);

            firingSolution = PnPUtils.getPitchAndYaw(input, boundingRect, cropCopy, fov);

            pitch = firingSolution[0];

            double goalWallDist = (horDist + verDist)/2;

            double goalWallDist2 = BetterTowerGoalUtils.getDistanceToGoalWall(14, firingSolution[0] + pitchOffset);
            powershots = BetterTowerGoalUtils.approxPowershotAngles(-firingSolution[1], goalWallDist2);

            firingSolution[0] = goalWallDist2;

            DecimalFormat format = new DecimalFormat("#.##");
            Imgproc.putText(cropCopy, "Firing Solution: ", new Point(2, input.height() - 170), 1, 2, new Scalar(0, 0, 0));
            //Imgproc.putText(cropCopy, "Pitch " + format.format(tmp[0]), new Point(2, input.height() - 100), 1, 3, new Scalar(0, 0, 0));
            Imgproc.putText(cropCopy, "Range " + format.format((goalWallDist2)), new Point(2, input.height() - 100), 1, 3, new Scalar(0, 0, 0));
            Imgproc.putText(cropCopy, "Heading " + format.format(firingSolution[1]), new Point(2, input.height() - 10), 1, 2.75, new Scalar(0, 0, 0));
            //Imgproc.putText(cropCopy, "Err " + format.format(horDist-verDist), new Point(2, input.height() - 10), 1, 3, new Scalar(0, 0, 0));

            double xDist = BetterTowerGoalUtils.approximateGoalX(goalWallDist2, firingSolution[1]);
            position = new double[]{xDist, goalWallDist2};
            track.set(true);
        }else{
            track.set(false);
        }

        m.release();

        //saveMatToDisk(cropCopy, "out" + idx);
        //idx ++;

        return cropCopy;
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