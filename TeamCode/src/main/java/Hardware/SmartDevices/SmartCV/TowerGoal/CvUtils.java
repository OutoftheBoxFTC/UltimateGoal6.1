package Hardware.SmartDevices.SmartCV.TowerGoal;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.TermCriteria;
import org.opencv.imgproc.Imgproc;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

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

    public static Mat crop(Mat in, RotatedRect rect){
        Point center = rect.center;
        Size size = rect.size;
        double theta = rect.angle;

        Mat rotMat = Imgproc.getRotationMatrix2D(center, theta, 1);

        Mat dst = new Mat();
        Imgproc.warpAffine(in, dst, rotMat, in.size());

        Mat out = new Mat();
        Imgproc.getRectSubPix(dst, size, center, out);
        return out;
    }

    public static Mat crop(Mat in, RotatedRect rect, int padding){
        Point center = rect.center;
        Size size = enlargeROI(in, rect.boundingRect(), padding).size();
        double theta = rect.angle;

        Mat rotMat = Imgproc.getRotationMatrix2D(center, theta, 1);

        Mat dst = new Mat();
        Imgproc.warpAffine(in, dst, rotMat, in.size());

        Mat out = new Mat();
        Imgproc.getRectSubPix(dst, size, center, out);
        return out;
    }

    public static Scalar getRandomColor(){
        return new Scalar(Math.random() * 255, Math.random() * 255, Math.random() * 255);
    }

    public static MatOfPoint refineCorners(Mat in, MatOfPoint corners){
        int[] cornersData = new int[(int) (corners.total() * corners.channels())];
        corners.get(0, 0, cornersData);
        int radius = 4;
        Mat matCorners = new Mat(corners.rows(), 2, CvType.CV_32F);
        float[] matCornersData = new float[(int) (matCorners.total() * matCorners.channels())];
        matCorners.get(0, 0, matCornersData);
        for (int i = 0; i < corners.rows(); i++) {
            matCornersData[i * 2] = cornersData[i * 2];
            matCornersData[i * 2 + 1] = cornersData[i * 2 + 1];
        }
        matCorners.put(0, 0, matCornersData);

        Size winSize = new Size(25, 25);
        Size zeroZone = new Size(-1, -1);
        TermCriteria criteria = new TermCriteria(TermCriteria.EPS + TermCriteria.COUNT, 40, 0.001);
        Imgproc.cornerSubPix(in, matCorners, winSize, zeroZone, criteria);

        List<Point> pts = new ArrayList<>();
        for(int i = 0; i < corners.rows(); i ++){
            pts.add(new Point(matCornersData[i * 2], matCornersData[i * 2 + 1]));
        }
        return new MatOfPoint(pts.toArray(new Point[1]));
    }

    public static Point getCenter(Rect rect){
        if (rect == null) {
            return new Point(0, 0);
        }
        return new Point(rect.x + rect.width / 2.0, rect.y + rect.height / 2.0);
    }

    public static void printLoadingBar(String label, int i, double length){
        DecimalFormat format = new DecimalFormat("##.##");
        int progress = (int) ((i/length) * 20);

        StringBuilder s = new StringBuilder(label + " ");
        s.append(format.format((i/length) * 100)).append("% [");
        for(int j = 0; j < 20; j ++){
            if(j < progress) {
                s.append("=");
            }else if (j == progress){
                s.append(">");
            }else{
                s.append(" ");
            }
        }
        s.append("] ").append(i).append("/").append((int)length);

        System.out.print(s.toString() + "\r");
    }

    public static void drawRRect(Mat in, RotatedRect rect, Scalar color, int thickness){
        Point[] points = new Point[4];
        rect.points(points);
        for(int i=0; i<4; ++i){
            Imgproc.line(in, points[i], points[(i+1)%4], color, thickness);
        }
    }

    public static double calcPinholeHor(double fov, double imageWidth, double imageHeight){
        double diagonalView = Math.toRadians(fov);
        Fraction aspectFraction = new Fraction(imageWidth, imageHeight);
        int horizontalRatio = aspectFraction.getNumerator();
        int verticalRatio = aspectFraction.getDenominator();
        double diagonalAspect = Math.hypot(horizontalRatio, verticalRatio);
        double horizontalView = Math.atan(Math.tan(diagonalView / 2) * (horizontalRatio / diagonalAspect)) * 2;

        return imageWidth / (2 * Math.tan(horizontalView / 2));
    }

    public static double calcPinholeVert(double fov, double imageWidth, double imageHeight){
        double diagonalView = Math.toRadians(fov);
        Fraction aspectFraction = new Fraction(imageWidth, imageHeight);
        int horizontalRatio = aspectFraction.getNumerator();
        int verticalRatio = aspectFraction.getDenominator();
        double diagonalAspect = Math.hypot(horizontalRatio, verticalRatio);
        double verticalView = Math.atan(Math.tan(diagonalView / 2) * (verticalRatio / diagonalAspect)) * 2;

        return imageHeight / (2 * Math.tan(verticalView / 2));
    }

    public static Rect cleanRect(Rect in, Mat mat){
        Point tl = in.clone().tl();
        Point br = in.clone().br();
        if(tl.x < 0){
            tl.x = 0;
        }
        if(tl.y < 0){
            tl.y = 0;
        }
        if(br.x < 0){
            br.x = 0;
        }
        if(br.y < 0){
            br.y = 0;
        }
        if(tl.x > mat.cols()){
            tl.x = mat.cols();
        }
        if(tl.y > mat.rows()){
            tl.y = mat.rows();
        }
        if(br.x > mat.cols()){
            br.x = mat.cols();
        }
        if(br.y > mat.rows()){
            br.y = mat.rows();
        }
        return new Rect(tl, br);
    }
}
