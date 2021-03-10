package Hardware.SmartDevices.SmartCV.Utils;

import android.graphics.Bitmap;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;

public class CvUtils {

    public static Rect enlargeROI(Mat frm, Rect boundingBox, int padding) {
        Rect returnRect = new Rect(boundingBox.x - padding, boundingBox.y - padding, boundingBox.width + (padding * 2), boundingBox.height + (padding * 2));
        if (returnRect.x < 0)returnRect.x = 0;
        if (returnRect.y < 0)returnRect.y = 0;
        if (returnRect.x+returnRect.width >= frm.cols())returnRect.width = frm.cols()-returnRect.x;
        if (returnRect.y+returnRect.height >= frm.rows())returnRect.height = frm.rows()-returnRect.y;
        return returnRect;
    }

    public static Point rotatePoint(Point p, Point center, double angle){
        double newX = center.x + (p.x - center.x) * Math.cos(angle) - (p.y - center.y) * Math.sin(angle);
        double newY = center.y + (p.x - center.x) * Math.sin(angle) - (p.y - center.y) * Math.cos(angle);
        return new Point(newX, newY);
    }

    public static Mat toPosition(Mat tvec, Mat rvec) {
        Mat rod = new Mat();

        Calib3d.Rodrigues(rvec, rod);

        Mat transpose = new Mat();

        Core.transpose(rod, transpose);

        Mat inversed = new Mat();

        Core.scaleAdd(transpose, -1, Mat.zeros(transpose.size(), transpose.type()), inversed);

        Mat multiplyResult = new Mat();

        Core.gemm(inversed, tvec, 1, new Mat(), 0, multiplyResult);

        rod.release();
        transpose.release();
        inversed.release();

        return multiplyResult;
    }
}
