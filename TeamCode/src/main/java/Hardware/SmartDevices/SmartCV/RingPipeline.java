package Hardware.SmartDevices.SmartCV;

import com.qualcomm.robotcore.util.RobotLog;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class RingPipeline extends OpenCvPipeline {

    private int numRings = -1;
    private double area = -1;
    private ArrayList<MatOfPoint> contours = new ArrayList<>();

    @Override
    public Mat processFrame(Mat input) {
        RobotLog.ii("Processing Frames", "true");
        Mat processed = new Mat();
        Imgproc.cvtColor(input, processed, Imgproc.COLOR_RGB2HSV);

        Scalar min = new Scalar(6, 151, 99);
        Scalar max = new Scalar(23, 255, 255);

        Mat mask = new Mat();
        Core.inRange(processed, min, max, mask);

        contours.clear();

        Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);

        int largestIndex = 0;
        double maxSize = 0;

        if(contours.size() > 0) {

            for (int i = 0; i < contours.size(); i++) {
                double area = Imgproc.contourArea(contours.get(i));
                if (maxSize < area) {
                    maxSize = area;
                    largestIndex = i;
                }
            }

            MatOfPoint2f matchCurve = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(largestIndex).toArray()), matchCurve, 0.005 * Imgproc.arcLength(new MatOfPoint2f(contours.get(largestIndex).toArray()), true), true);

            Rect matchRect = Imgproc.boundingRect(matchCurve);
            if (matchRect.area() < 100) {
                numRings = 0;
            } else if (matchRect.area() < 2200) {
                numRings = 1;
            } else {
                numRings = 4;
            }

            area = matchRect.area();

            Imgproc.rectangle(input, matchRect, new Scalar(255, 0, 0), 3);

            processed.release();
            mask.release();
            for(MatOfPoint contour : contours){
                contour.release();
            }
            matchCurve.release();
        }else{
            numRings = 0;
            area = 0;
        }
        return input;
    }

    public int getNumRings() {
        return numRings;
    }

    public double getArea() {
        return area;
    }
}
