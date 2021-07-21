package Hardware.SmartDevices.SmartCV;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class RingMaskPipeline extends OpenCvPipeline {

    private int numRings = -1;
    private double area = -1;
    Mat processed = new Mat();
    Mat mask = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_BGR2YCrCb);
        //[0.0, 130.0, 0.0, 0.0] [111.0, 230.0, 150.0, 0.0]
        Mat frameHSV = new Mat();
        Imgproc.cvtColor(input, frameHSV, Imgproc.COLOR_BGR2YCrCb);
        Mat thresh = new Mat();
        Core.inRange(frameHSV, new Scalar(0, 135, 0), new Scalar(110, 230, 150), thresh);
        Imgproc.rectangle(thresh, new Rect(new Point(0, 0), new Point(640, 480 / 3.0)), Scalar.all(0), -1);

        Imgproc.GaussianBlur(thresh, thresh, new Size(15.0, 15.0), 0.00);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(thresh, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);

        Imgproc.drawContours(input, contours, -1, new Scalar(0, 255, 0), 3);

        double area = 0;

        for(MatOfPoint m : contours){
            if(Imgproc.contourArea(m) > 250){
                Imgproc.rectangle(input, Imgproc.boundingRect(m), new Scalar(255, 0, 0), 3);
                if(Imgproc.contourArea(m) > area){
                    area = Imgproc.contourArea(m);
                }
            }
        }

        if(area < 1000){
            numRings = 0;
        }else if(area < 3000){
            numRings = 1;
        }else if(area < 5000){
            numRings = 4;
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
