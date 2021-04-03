package Hardware.SmartDevices.SmartCV.TowerGoal;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.features2d.BRISK;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class TowerGoalUtils {
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

    public static MatOfPoint getContour(Mat inMat)  {
        Mat cannyOut = new Mat();
        Imgproc.Canny(inMat, cannyOut, 100, 200);

        List<MatOfPoint> referenceContours = new ArrayList<>();
        Imgproc.findContours(inMat, referenceContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        Imgproc.cvtColor(cannyOut, cannyOut, Imgproc.COLOR_GRAY2BGR);

        Imgproc.drawContours(cannyOut, referenceContours, 0, new Scalar(0, 0, 255));

        //HighGui.imshow("bruh" + Math.random(), cannyOut);

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
            MatOfPoint match = referenceContours.get(maxValIdx);
            return match;
        }else{
            return new MatOfPoint();
        }

    }

    public static double[] getRefMoments(Mat inMat){
        Mat hsv = new Mat();
        Imgproc.cvtColor(inMat, hsv, Imgproc.COLOR_RGB2HSV);
        //Imgproc.cvtColor(inMat, inMat, Imgproc.COLOR_RGB2GRAY);
        Imgproc.resize(hsv, hsv, new Size(inMat.width(), inMat.height())); //[112.0, 113.0, 132.0, 0.0] [122.0, 255.0, 255.0, 0.0]
        //[114.0, 134.0, 144.0, 0.0] [119.0, 255.0, 255.0, 0.0]
        Scalar min = new Scalar(114, 134, 144);
        Scalar max = new Scalar(119, 255, 255);
        Mat outMat = new Mat();
        Core.inRange(hsv, min, max, outMat);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(10, 10));
        Imgproc.morphologyEx(outMat, outMat, Imgproc.MORPH_CLOSE, kernel);

        Moments moments = Imgproc.moments(outMat);
        Mat hu = new Mat();
        Imgproc.HuMoments(moments, hu);
        double[] huArr = new double[7];
        hu.get(0, 0, huArr);

        for(int i = 0; i < huArr.length; i ++){
            huArr[i] = -1 * ((huArr[i])/Math.abs(huArr[i])) * Math.log10(Math.abs(huArr[i]));
        }

        return huArr;
    }

    public static Mat cropInRange(Mat inMat){
        Mat hsv = new Mat();
        Imgproc.cvtColor(inMat, hsv, Imgproc.COLOR_RGB2HSV);
        //Imgproc.cvtColor(inMat, inMat, Imgproc.COLOR_RGB2GRAY);
        Imgproc.resize(hsv, hsv, new Size(inMat.width(), inMat.height())); //[112.0, 113.0, 132.0, 0.0] [122.0, 255.0, 255.0, 0.0]
        //[114.0, 134.0, 144.0, 0.0] [119.0, 255.0, 255.0, 0.0]
        Scalar min = new Scalar(112, 113, 132);
        Scalar max = new Scalar(122, 255, 255);
        Mat outMat = new Mat();
        Core.inRange(hsv, min, max, outMat);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(10, 10));
        Imgproc.morphologyEx(outMat, outMat, Imgproc.MORPH_CLOSE, kernel);

        return outMat;
    }

    public static MatOfPoint getMatchRect(MatOfPoint refContour, Mat inMat, Mat toWrite){
        Mat hsv = new Mat();
        Imgproc.cvtColor(inMat, hsv, Imgproc.COLOR_RGB2HSV);
        //Imgproc.cvtColor(inMat, inMat, Imgproc.COLOR_RGB2GRAY);
        //Imgproc.resize(hsv, hsv, new Size(1280, (inMat.width() * (720/1280.0)))); //[112.0, 113.0, 132.0, 0.0] [122.0, 255.0, 255.0, 0.0]
        //[114.0, 134.0, 144.0, 0.0] [119.0, 255.0, 255.0, 0.0]
        //[112.0, 113.0, 132.0, 0.0] [122.0, 255.0, 255.0, 0.0]
        Scalar min = new Scalar(112, 113, 132);
        Scalar max = new Scalar(122, 255, 255);
        //Scalar min = new Scalar(0, 0, 0);
        //Scalar max = new Scalar(255, 50, 50);
        Mat outMat = new Mat();
        Core.inRange(hsv, min, max, outMat);


        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(10, 10));
        Imgproc.morphologyEx(outMat, outMat, Imgproc.MORPH_OPEN, kernel);

        kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(10, 10));
        Imgproc.morphologyEx(outMat, outMat, Imgproc.MORPH_CLOSE, kernel);

        outMat.copyTo(toWrite);

        Imgproc.cvtColor(toWrite, toWrite, Imgproc.COLOR_GRAY2BGR);

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

            double distance = Imgproc.matchShapes(croppedContour, refContour, Imgproc.CONTOURS_MATCH_I3, 0);
            //refineTowerCompare(contours.get(i), toWrite);
            MatOfPoint2f points = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(croppedContour.toArray()), points, 0.02 * (Imgproc.arcLength(new MatOfPoint2f(croppedContour.toArray()), true)), true);
            distance = Imgproc.matchShapes(new MatOfPoint(points.toArray()), refContour, Imgproc.CONTOURS_MATCH_I3, 0);

            Imgproc.approxPolyDP(new MatOfPoint2f(contour.toArray()), points, 0.02 * (Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true)), true);
            Imgproc.drawContours(toWrite, Collections.singletonList(new MatOfPoint(points.toArray())), -1, new Scalar(0, 0, 255), 3);
            rect = Imgproc.boundingRect((new MatOfPoint(points.toArray())));
            double aspect = ((double)(rect.height)) / ((double)rect.width);
            dpPoints.add(points);
            //System.out.println(distance + " | " + rect.area() + " | " + aspect + " | " + points.toArray().length);
            //if(distance < 0.2){
                //Imgproc.drawContours(toWrite, contours, i, new Scalar(0, 0, 255));
            //}
            double numMatch = matchPoints(new MatOfPoint2f(refContour.toArray()), points, matchRect, Imgproc.minAreaRect(points));
            //System.out.println(rect.area() + " | " + points.toArray().length + " | " + aspect + " | " + numMatch);
            if(distance < minDist && rect.area() > 1600 && points.toArray().length < 10 && points.toArray().length > 4){
                if(aspect > 0.56 && aspect < 2) {
                    if(numMatch >= 4) {
                        minDist = distance;
                        idx = i;
                    }
                }
            }
        }
        if(idx != -1){
            if(minDist < 0.25) {
                //Imgproc.putText(writeMat, "Confidence: " + dec.format(100-minDist), new Point(writeMat.width()/2, writeMat.height()/2), Imgproc.FONT_HERSHEY_COMPLEX, 0.6, new Scalar(0, 255, 0), 1);
                //System.out.print(minDist);
                //Imgproc.drawContours(toWrite, contours, idx, new Scalar(255, 0, 0), 5);
                //System.out.println("e");
                return new MatOfPoint(dpPoints.get(idx).toArray());
            }
        }
        return new MatOfPoint();
        //HighGui.imshow("Test", toWrite);
        //HighGui.waitKey();
    }

    public static MatOfPoint getMatchRect(MatOfPoint refContour, Mat inMat){
        Mat hsv = new Mat();
        Imgproc.cvtColor(inMat, hsv, Imgproc.COLOR_RGB2HSV);
        Scalar min = new Scalar(112, 113, 132);
        Scalar max = new Scalar(122, 255, 255);

        Mat outMat = new Mat();
        Core.inRange(hsv, min, max, outMat);

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
            Imgproc.approxPolyDP(new MatOfPoint2f(croppedContour.toArray()), points, 0.02 * (Imgproc.arcLength(new MatOfPoint2f(croppedContour.toArray()), true)), true);
            double distance = Imgproc.matchShapes(new MatOfPoint(points.toArray()), refContour, Imgproc.CONTOURS_MATCH_I3, 0);

            Imgproc.approxPolyDP(new MatOfPoint2f(contour.toArray()), points, 0.02 * (Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true)), true);

            rect = Imgproc.boundingRect((new MatOfPoint(points.toArray())));
            double aspect = ((double)(rect.height)) / ((double)rect.width);
            dpPoints.add(points);

            double numMatch = matchPoints(new MatOfPoint2f(refContour.toArray()), points, matchRect, Imgproc.minAreaRect(points));

            if(distance < minDist && rect.area() > 1600 && points.toArray().length < 10 && points.toArray().length > 4){
                if(aspect > 0.56 && aspect < 2) {
                    if(numMatch >= 4) {
                        minDist = distance;
                        idx = i;
                    }
                }
            }

            contour.release();
            cropped.release();
            croppedContour.release();
            points.release();
        }

        hsv.release();
        outMat.release();
        kernel.release();
        contours.clear();
        dpPoints.clear();

        if(idx != -1){
            if(minDist < 0.25) {
                return new MatOfPoint(dpPoints.get(idx).toArray());
            }
        }
        return new MatOfPoint();
    }

    public static MatOfKeyPoint getKeypoints(Mat in){
        MatOfKeyPoint kps = new MatOfKeyPoint();
        BRISK brisk = BRISK.create();
        brisk.detect(in, kps);
        return kps;
    }

    public static Mat getDescriptors(Mat in){
        MatOfKeyPoint kps = new MatOfKeyPoint();
        Mat desc = new Mat();
        BRISK brisk = BRISK.create(50);
        brisk.detectAndCompute(in, new Mat(), kps, desc);
        return desc;
    }

    public static double matchPoints(MatOfPoint2f ref, MatOfPoint2f match, Rect refRect, RotatedRect matchRect){
        double epsilon = 0.15;

        int numMatch = 0;
        for(Point p : ref.toArray()) {
            for(Point pt : match.toArray()) {
                Point scaledRef = new Point((p.x - refRect.x)/refRect.width, (p.y - refRect.y)/refRect.height);
                Point scaledMatch = new Point((pt.x - matchRect.boundingRect().x)/matchRect.boundingRect().width, (pt.y - matchRect.boundingRect().y)/matchRect.boundingRect().height);
                Point rotScaledMatch = CvUtils.rotatePoint(scaledMatch, matchRect.center, matchRect.angle);

                double xDelta = scaledMatch.x - scaledRef.x;
                double yDelta = scaledMatch.y - scaledRef.y;
                double delta = Math.sqrt((xDelta * xDelta) + (yDelta * yDelta));
                //System.out.println(p + " | " + pt + " | " + scaledRef + " | " + scaledMatch + " | " + delta);
                if(delta < epsilon){
                    numMatch ++;
                    break;
                }
            }
            //System.out.println("=========================");
        }
        //System.out.println(numMatch);
        return numMatch;
    }

    public static double matchPoints(MatOfPoint2f ref, MatOfPoint2f match, Rect refRect, RotatedRect matchRect, MatOfPoint3f objectPoints, MatOfPoint2f imagePoints){
        double epsilon = 0.01;

        int numMatch = 0;
        for(Point p : ref.toArray()) {
            for(Point pt : match.toArray()) {
                Point scaledRef = new Point((p.x - refRect.x)/refRect.width, (p.y - refRect.y)/refRect.height);
                Point scaledMatch = new Point((pt.x - matchRect.boundingRect().x)/matchRect.boundingRect().width, (pt.y - matchRect.boundingRect().y)/matchRect.boundingRect().height);
                Point rotScaledMatch = CvUtils.rotatePoint(scaledMatch, matchRect.center, matchRect.angle);

                double xDelta = scaledMatch.x - scaledRef.x;
                double yDelta = scaledMatch.y - scaledRef.y;
                double delta = Math.sqrt((xDelta * xDelta) + (yDelta * yDelta));
                if(delta < epsilon){
                    numMatch ++;
                    float sclX = (float)(((p.x/(refRect.width + refRect.x)) * 23.875) - (23.875/2));
                    float sclY = (float)(((p.y/(refRect.height + refRect.y)) * 15.75) - (15.75/2));
                    objectPoints.push_back(new MatOfPoint3f(new Point3(sclX, sclY, 0)));
                    imagePoints.push_back(new MatOfPoint2f(pt));
                    break;
                }
            }
            //System.out.println("=========================");
        }
        System.out.println(numMatch);
        return numMatch;
    }

    public static void getPos(MatOfPoint2f ref, MatOfPoint2f match, Rect refRect, RotatedRect matchRect, Mat toDraw){
        MatOfPoint3f objectPoints = new MatOfPoint3f();
        MatOfPoint2f imagePoints = new MatOfPoint2f();

        double numMatch = matchPoints(ref, match, refRect, matchRect, objectPoints, imagePoints);
        if(numMatch > 4){
            MatOfDouble distCoeffs = new MatOfDouble();
            Mat intrinsic = new Mat(3, 3, CvType.CV_32FC1);
            Mat rvec = new Mat();
            Mat tvec = new Mat();

            double[] inrinsicFloat = new double[] {817.063304531327,0.0,325.9485286458284,0.0,819.4690054531818,236.2597899599986,0.0,0.0,1.0,0.0};
            intrinsic.put(0, 0, inrinsicFloat);

            double[] distFloat = new double[] {-0.014680796227423968,1.3720322590501144,-0.0028429009326778093,0.0010064951672061734,-5.347658630748131};
            distCoeffs.fromArray(distFloat);

            boolean solved = Calib3d.solvePnP(objectPoints, imagePoints, intrinsic, distCoeffs, rvec, tvec);
            //System.out.println(solved);
            System.out.println(CvUtils.toPosition(tvec, rvec).dump());
            Calib3d.drawFrameAxes(toDraw, intrinsic, distCoeffs, rvec, tvec, 5, 3);
        }
    }
}
