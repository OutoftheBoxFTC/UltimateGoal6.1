package Hardware.SmartDevices.SmartCV;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class RingMaskPipeline extends OpenCvPipeline {

    private int numRings = -1;
    private double area = -1;
    Mat processed = new Mat();
    Mat mask = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        input = input.submat(new Rect(0, (int) (input.height()-(input.height()/2.75)), input.width(), input.height()/4));
        /**Imgproc.cvtColor(input, processed, Imgproc.COLOR_RGB2HSV);

        Scalar min = new Scalar(6, 100, 99);
        Scalar max = new Scalar(23, 255, 255);
        */
        Imgproc.cvtColor(input, processed, Imgproc.COLOR_RGB2YCrCb);

        Scalar min = new Scalar(0, 150, 0);
        Scalar max = new Scalar(150, 170, 125);
        Core.inRange(processed, min, max, mask);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(1, 1));
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);

        ArrayList<MatOfPoint> contours = new ArrayList<>();

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
            } else if (matchRect.area() < 2200) {
                numRings = 1;
            } else {
                numRings = 4;
            }

            area = matchRect.area();

            Mat edited = new Mat();
            Core.copyTo(input, edited, mask);

            Imgproc.rectangle(mask, matchRect, new Scalar(255, 0, 0), 4);

            return edited;
        }else{
            numRings = 0;
            area = 0;
            return input;
        }
    }

    public int getNumRings() {
        return numRings;
    }

    public double getArea() {
        return area;
    }
}
