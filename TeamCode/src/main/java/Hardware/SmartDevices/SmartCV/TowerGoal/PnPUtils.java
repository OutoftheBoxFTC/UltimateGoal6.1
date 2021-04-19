package Hardware.SmartDevices.SmartCV.TowerGoal;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.TermCriteria;
import org.opencv.imgproc.Imgproc;

public class PnPUtils {

    public static double[] getPitchAndYaw(Mat in, Rect bounds, Mat outMat, double fov){

        Mat inCopy = in.submat(bounds);
        Mat inNorm = BetterTowerGoalUtils.normCropInRange(inCopy);

        double width = (in.width()/2.0) - 0.5;
        double height = (in.height()/2.0) - 0.5;

        Point center = CvUtils.getCenter(bounds);
        double pitch = -Math.toDegrees(Math.atan2(center.y - height, CvUtils.calcPinholeVert(fov, in.width(), in.height())));
        double yaw = Math.toDegrees(Math.atan2((center.x) - width, CvUtils.calcPinholeHor(fov, in.width(), in.height())));
        //System.out.println("Center " + center + " | vert " + CvUtils.calcPinholeVert(fov, in.width(), in.height()) + " | hor " + CvUtils.calcPinholeHor(fov, in.width(), in.height()));
        //System.out.println("Pitch " + pitch + " | Yaw " + yaw);

        //Imgproc.rectangle(outMat, bounds, new Scalar(0, 0, 255));

        return new double[]{ pitch, yaw };
    }

    public static Mat getPitchAndYaw(Mat in, Point center){

        Mat outMat = in.clone();
        Imgproc.cvtColor(outMat, outMat, Imgproc.COLOR_BGR2GRAY);
        Imgproc.cvtColor(outMat, outMat, Imgproc.COLOR_GRAY2BGR);

        double width = (in.width()/2.0) - 0.5;
        double height = (in.height()/2.0) - 0.5;
        double fov = 123;
        //fov = 180;

        double pitch = -Math.toDegrees(Math.atan2(center.y - height, CvUtils.calcPinholeVert(fov, in.width(), in.height())));
        double yaw = Math.toDegrees(Math.atan2(center.x - width, CvUtils.calcPinholeHor(fov, in.width(), in.height())));
        System.out.println("Center " + center + " | vert " + CvUtils.calcPinholeVert(fov, in.width(), in.height()) + " | hor " + CvUtils.calcPinholeHor(fov, in.width(), in.height()));
        System.out.println("Pitch " + pitch + " | Yaw " + yaw);

        return outMat;
    }
}
