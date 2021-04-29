package Hardware.SmartDevices.SmartCV.TowerGoal;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

import Hardware.SmartDevices.SmartCV.TowerGoal.CvUtils;

public class BetterTowerGoalUtils {

    public static Mat cropInRange(Mat inMat){
        Mat hsv = new Mat();
        Imgproc.cvtColor(inMat, hsv, Imgproc.COLOR_RGB2HSV);
        //Imgproc.cvtColor(inMat, inMat, Imgproc.COLOR_RGB2GRAY);
        Imgproc.resize(hsv, hsv, new Size(inMat.width(), inMat.height())); //[112.0, 113.0, 132.0, 0.0] [122.0, 255.0, 255.0, 0.0]
        //[114.0, 134.0, 144.0, 0.0] [119.0, 255.0, 255.0, 0.0]
        Scalar min = new Scalar(106, 80, 100);
        //Scalar max = new Scalar(122, 255, 255);
        Scalar max = new Scalar(128, 255, 255);
        Mat outMat = new Mat();
        Core.inRange(hsv, min, max, outMat);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(10, 10));
        Imgproc.morphologyEx(outMat, outMat, Imgproc.MORPH_OPEN, kernel);

        hsv.release();

        return outMat;
    }

    public static Mat normCropInRange(Mat inMat){
        Mat hsv = new Mat();
        Imgproc.cvtColor(inMat, hsv, Imgproc.COLOR_RGB2HSV);
        //Imgproc.cvtColor(inMat, inMat, Imgproc.COLOR_RGB2GRAY);
        //Imgproc.resize(hsv, hsv, new Size(inMat.width(), inMat.height())); //[112.0, 113.0, 132.0, 0.0] [122.0, 255.0, 255.0, 0.0]
        //[114.0, 134.0, 144.0, 0.0] [119.0, 255.0, 255.0, 0.0]
        //Scalar min = new Scalar(112, 113, 132);
        Scalar min = new Scalar(106, 80, 100);
        //Scalar max = new Scalar(122, 255, 255);
        Scalar max = new Scalar(128, 255, 255);
        Mat outMat = new Mat();
        Core.inRange(hsv, min, max, outMat);

        hsv.release();

        return outMat;
    }

    public static MatOfPoint getReference(Mat referenceMat)  {
        Mat grayReferenceMat = new Mat();
        Imgproc.cvtColor(referenceMat, grayReferenceMat, Imgproc.COLOR_BGR2GRAY);

        Imgproc.resize(grayReferenceMat, grayReferenceMat, new Size(500, 500));

        Mat cannyOut = new Mat();
        Imgproc.Canny(grayReferenceMat, cannyOut, 100, 200);

        List<MatOfPoint> referenceContours = new ArrayList<>();
        Imgproc.findContours(cannyOut, referenceContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        Imgproc.cvtColor(cannyOut, cannyOut, Imgproc.COLOR_GRAY2BGR);

        double maxVal = 0;
        int maxValIdx = 0;
        for (int contourIdx = 0; contourIdx < referenceContours.size(); contourIdx++)
        {
            double contourArea = Imgproc.contourArea(referenceContours.get(contourIdx));
            if (maxVal < contourArea)
            {
                maxVal = contourArea;
                maxValIdx = contourIdx;
            }
        }

        MatOfPoint match = referenceContours.get(maxValIdx);

        MatOfPoint2f contourMat = new MatOfPoint2f(match.toArray());
        MatOfPoint2f poly = new MatOfPoint2f();
        Imgproc.approxPolyDP(contourMat, poly, 0.01 * Imgproc.arcLength(contourMat, true), true);

        MatOfPoint toReturn = new MatOfPoint(poly.toArray());

        grayReferenceMat.release();
        cannyOut.release();
        referenceContours.clear();
        match.release();
        contourMat.release();
        poly.release();

        return toReturn;
    }

    public static MatOfPoint getNormReference(Mat referenceMat)  {
        Mat grayReferenceMat = new Mat();
        Imgproc.cvtColor(referenceMat, grayReferenceMat, Imgproc.COLOR_BGR2GRAY);

        Imgproc.resize(grayReferenceMat, grayReferenceMat, new Size(500, 500));

        Mat cannyOut = new Mat();
        Imgproc.Canny(grayReferenceMat, cannyOut, 100, 200);

        List<MatOfPoint> referenceContours = new ArrayList<>();
        Imgproc.findContours(cannyOut, referenceContours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_NONE);

        Imgproc.cvtColor(cannyOut, cannyOut, Imgproc.COLOR_GRAY2BGR);

        double maxVal = 0;
        int maxValIdx = 0;
        for (int contourIdx = 0; contourIdx < referenceContours.size(); contourIdx++)
        {
            double contourArea = Imgproc.contourArea(referenceContours.get(contourIdx));
            if (maxVal < contourArea)
            {
                maxVal = contourArea;
                maxValIdx = contourIdx;
            }
        }

        MatOfPoint match = referenceContours.get(maxValIdx);

        grayReferenceMat.release();
        cannyOut.release();
        referenceContours.clear();

        return match;
    }

    public static MatOfPoint getContour(Mat inMat)  {
        List<MatOfPoint> referenceContours = new ArrayList<>();
        Imgproc.findContours(inMat, referenceContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        double maxVal = 0;
        int maxValIdx = 0;
        for (int contourIdx = 0; contourIdx < referenceContours.size(); contourIdx++)
        {
            double contourArea = Imgproc.contourArea(referenceContours.get(contourIdx));
            if (maxVal < contourArea)
            {
                maxVal = contourArea;
                maxValIdx = contourIdx;
            }
        }

        if(referenceContours.size() > 0) {
            MatOfPoint toReturn = new MatOfPoint(referenceContours.get(maxValIdx).toArray());
            for(MatOfPoint m : referenceContours){
                m.release();
            }
            return toReturn;
        }else{
            return new MatOfPoint();
        }
    }

    public static MatOfPoint getNormContour(Mat inMat)  {
        List<MatOfPoint> referenceContours = new ArrayList<>();
        Imgproc.findContours(inMat, referenceContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);

        double maxVal = 0;
        int maxValIdx = 0;
        for (int contourIdx = 0; contourIdx < referenceContours.size(); contourIdx++)
        {
            double contourArea = Imgproc.contourArea(referenceContours.get(contourIdx));
            if (maxVal < contourArea)
            {
                maxVal = contourArea;
                maxValIdx = contourIdx;
            }
        }

        if(referenceContours.size() > 0) {
            MatOfPoint toReturn = new MatOfPoint(referenceContours.get(maxValIdx).toArray());
            for(MatOfPoint m : referenceContours){
                m.release();
            }
            return toReturn;
        }else{
            return new MatOfPoint();
        }
    }

    public static MatOfPoint getChainedContour(Mat inMat)  {
        List<MatOfPoint> referenceContours = new ArrayList<>();
        Imgproc.findContours(inMat, referenceContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        double maxVal = 0;
        int maxValIdx = 0;
        for (int contourIdx = 0; contourIdx < referenceContours.size(); contourIdx++)
        {
            double contourArea = Imgproc.contourArea(referenceContours.get(contourIdx));
            if (maxVal < contourArea)
            {
                maxVal = contourArea;
                maxValIdx = contourIdx;
            }
        }

        if(referenceContours.size() > 0) {
            MatOfPoint toReturn = new MatOfPoint(referenceContours.get(maxValIdx).toArray());
            for(MatOfPoint m : referenceContours){
                m.release();
            }
            return toReturn;
        }else{
            return new MatOfPoint();
        }
    }

    public static MatOfPoint getMatchRect(MatOfPoint refContour, Mat inMat){
        //Mat resized = new Mat();
        //Imgproc.resize(inMat, resized, new Size(640, 480));

        Mat outMat = cropInRange(inMat);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(10, 10));
        Imgproc.morphologyEx(outMat, outMat, Imgproc.MORPH_OPEN, kernel);

        kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(10, 10));
        Imgproc.morphologyEx(outMat, outMat, Imgproc.MORPH_CLOSE, kernel);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(outMat, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);

        int idx = -1;
        double minDist = Double.MAX_VALUE;

        ArrayList<MatOfPoint2f> dpPoints = new ArrayList<>();

        Rect matchRect = Imgproc.boundingRect(refContour);

        for(int i = 0; i < contours.size(); i ++){
            MatOfPoint contour = contours.get(i);
            Rect rect = Imgproc.boundingRect(contour);
            rect = CvUtils.enlargeROI(outMat, rect, 10);
            Mat cropped = outMat.submat(rect);

            Imgproc.resize(cropped, cropped, new Size(500, 500));

            MatOfPoint croppedContour = getContour(cropped);

            MatOfPoint2f points = new MatOfPoint2f();

            MatOfPoint2f croppedContour2f = new MatOfPoint2f(croppedContour.toArray());
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            MatOfPoint2f refContour2f = new MatOfPoint2f(refContour.toArray());

            Imgproc.approxPolyDP(croppedContour2f, points, 0.02 * (Imgproc.arcLength(croppedContour2f, true)), true);
            MatOfPoint pointsNorm = new MatOfPoint(points.toArray());

            double distance = Imgproc.matchShapes(pointsNorm, refContour, Imgproc.CONTOURS_MATCH_I3, 0);

            Imgproc.approxPolyDP(contour2f, points, 0.02 * (Imgproc.arcLength(contour2f, true)), true);

            rect = Imgproc.boundingRect((pointsNorm));
            double aspect = ((double)(rect.height)) / ((double)rect.width);
            dpPoints.add(points);

            double numMatch = matchPoints(refContour2f, points, matchRect, Imgproc.minAreaRect(points));

            //System.out.println(distance + " | " + rect.area() + " | " + points.toArray().length + " | " + aspect + " | " + numMatch);

            if(distance < minDist && rect.area() > 100000 && points.toArray().length < 10 && points.toArray().length > 4){
                if(aspect > 0.56 && aspect < 2) {
                    if(numMatch > 4) {
                        //System.out.println("up");
                        minDist = distance;
                        idx = i;
                    }
                }
            }

            contour.release();
            cropped.release();
            croppedContour.release();
            pointsNorm.release();
            croppedContour2f.release();
            contour2f.release();
            refContour2f.release();
        }

        outMat.release();
        kernel.release();
        contours.clear();

        MatOfPoint toReturn = new MatOfPoint();
        if(idx != -1){
            if(minDist < 0.2) {
                toReturn.release();
                toReturn = new MatOfPoint(dpPoints.get(idx).toArray());
                for(MatOfPoint2f mp2f : dpPoints){
                    mp2f.release();
                }
                dpPoints.clear();
            }
        }

        return toReturn;
    }

    public static double matchPoints(MatOfPoint2f ref, MatOfPoint2f match, Rect refRect, RotatedRect matchRect){
        double epsilon = 0.175;

        int numMatch = 0;
        for(Point p : ref.toArray()) {
            for(Point pt : match.toArray()) {
                Point scaledRef = new Point((p.x - refRect.x)/refRect.width, (p.y - refRect.y)/refRect.height);
                Point scaledMatch = new Point((pt.x - matchRect.boundingRect().x)/matchRect.boundingRect().width, (pt.y - matchRect.boundingRect().y)/matchRect.boundingRect().height);

                double xDelta = scaledMatch.x - scaledRef.x;
                double yDelta = scaledMatch.y - scaledRef.y;
                double delta = Math.sqrt((xDelta * xDelta) + (yDelta * yDelta));
                if(delta < epsilon){
                    numMatch ++;
                    break;
                }
            }
        }
        return numMatch;
    }

    public static MatOfPoint refineEstimation(Mat inMat, Rect boundingRect, Mat outMat){
        Mat inClone = inMat.clone();
        Rect enlarged = CvUtils.enlargeROI(inClone, boundingRect, 10);

        Mat cropped = inClone.submat(enlarged);

        Mat extracted = normCropInRange(cropped);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(extracted, extracted, Imgproc.MORPH_OPEN, kernel);

        kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        //Imgproc.morphologyEx(extracted, extracted, Imgproc.MORPH_CLOSE, kernel);
        //Core.extractChannel(cropped, extracted, 2);
        //Imgproc.adaptiveThreshold(extracted, extracted, 255, Imgproc.ADAPTIVE_THRESH_GAUSSIAN_C, Imgproc.THRESH_BINARY, 11, 2);

        //Imgproc.cvtColor(extracted, outMat, Imgproc.COLOR_GRAY2BGR);

        MatOfPoint contour = getContour(extracted);
        MatOfPoint2f cnt2f = new MatOfPoint2f(contour.toArray());
        if(cnt2f.toArray().length > 0) {
            MatOfPoint2f dp = new MatOfPoint2f();
            Imgproc.approxPolyDP(cnt2f, dp, 0.01 * Imgproc.arcLength(cnt2f, true), true);
            //contour = new MatOfPoint(dp.toArray());
            dp.release();
        }

        MatOfPoint scl = new MatOfPoint();
        for(Point p : contour.toArray()){
            Point tmp = new Point(p.x + enlarged.x, p.y + enlarged.y);
            scl.push_back(new MatOfPoint(tmp));
        }

        Mat test = inMat.clone();
        test = normCropInRange(test);
        //test.copyTo(outMat);
        test.release();

        inClone.release();
        cropped.release();
        extracted.release();
        contour.release();

        return scl;
    }

    public static double approximateDistanceToGoal(double objDim, double imgDim, double focalDist){
        return (objDim * focalDist) / imgDim;
    }

    public static double getDistanceToGoalWall(double cameraHeight, double pitch){
        return (40.625 - cameraHeight) / Math.tan(Math.toRadians(pitch));
    }

    public static double approximateCameraAngle(double cameraHeight, double dist, double pitch){
        double ang = Math.atan2((40.625 - cameraHeight), dist);
        return Math.toDegrees(ang) - pitch;
    }

    public static double[] approxPowershotAngles(double yaw, double goalWallDist){
        double goalOffset = 16.5;
        double powershotOffset = 7.5;

        double dist = Math.tan(Math.toRadians(yaw)) * goalWallDist;

        double psht1 = goalOffset;
        double psht2 = psht1 + powershotOffset;
        double psht3 = psht2 + powershotOffset;

        double[] dists = new double[3];

        if((psht1 > Math.abs(dist) && (psht1 * dist < 0)) || yaw > 0){
            dists[0] = Math.toDegrees(Math.atan((psht1 + dist) / goalWallDist));
        }else{
            dists[0] = Math.toDegrees(Math.atan((psht1 - dist) / goalWallDist));
        }

        if((psht2 > Math.abs(dist) && (psht2 * dist < 0)) || yaw > 0){
            dists[1] = Math.toDegrees(Math.atan((psht2 + dist) / goalWallDist));
        }else{
            dists[1] = Math.toDegrees(Math.atan((psht2 - dist) / goalWallDist));
        }

        if((psht3 > Math.abs(dist) && (psht3 * dist < 0)) || yaw > 0){
            dists[2] = Math.toDegrees(Math.atan((psht3 + dist) / goalWallDist));
        }else{
            dists[2] = Math.toDegrees(Math.atan((psht3 - dist) / goalWallDist));
        }

        return dists;
    }

    public static double approximateGoalX(double goalWallDist, double yaw){
        return goalWallDist * Math.tan(Math.toRadians(yaw));
    }
}
