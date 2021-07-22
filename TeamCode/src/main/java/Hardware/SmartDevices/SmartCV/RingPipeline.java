package Hardware.SmartDevices.SmartCV;

import com.qualcomm.robotcore.util.RobotLog;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

import static org.opencv.core.CvType.CV_32F;

public class RingPipeline extends OpenCvPipeline {

    private int numRings = -1;
    private double area = -1;
    private ArrayList<MatOfPoint> contours = new ArrayList<>();

    @Override
    public Mat processFrame(Mat input) {
        /**double[] intrinsic = new double[]{1786.922881201778, 0, 318.1071998135541, 0, 1817.4582296, 242.92967527, 0, 0, 1};
        double[] dist = new double[]{-14.2733, 129.931969, 0.04461513, -0.0025887199, -1718.9375484};
        Mat intrinsicMat = new Mat(3, 3, CV_32F);
        Mat distMat = new Mat(1, 5, CV_32F);
        intrinsicMat.put(0, 0, intrinsic);
        distMat.put(0, 0, dist);
        Mat undist = new Mat();
        Calib3d.undistort(input, undist, intrinsicMat, distMat);
        input = undist;
        */
        Mat clone = input.clone();
        //input = input.submat(new Rect(0, (int) (input.height()-(input.height()/2.5)), input.width(), input.height()/4));
        Mat processed = new Mat();
        Imgproc.cvtColor(input, processed, Imgproc.COLOR_RGB2YCrCb);

        Scalar min = new Scalar(0, 141, 0);
        Scalar max = new Scalar(255, 230, 95);

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
            if (matchRect.area() < 500) {
                numRings = 0;
            } else if (matchRect.area() < 9000) {
                numRings = 1;
            } else {
                numRings = 4;
            }
            RobotLog.ii("Area", "" + matchRect.area());

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
        return clone;
    }

    public int getNumRings() {
        return numRings;
    }

    public double getArea() {
        return area;
    }
}
