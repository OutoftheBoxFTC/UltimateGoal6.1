package Hardware.SmartDevices.SmartCV.Utils;

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
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class BetterTowerGoalUtils {

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

        hsv.release();
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

    public static Mat solvePnP(Mat in, RotatedRect r, Mat ref){
        long start = System.currentTimeMillis();
        Mat tst = BetterTowerGoalUtils.cropInRange(in);
        Imgproc.cvtColor(tst, tst, Imgproc.COLOR_GRAY2BGR);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(10, 10));
        Imgproc.morphologyEx(tst, tst, Imgproc.MORPH_OPEN, kernel);

        Rect adjusted = CvUtils.enlargeROI(tst, r.boundingRect(), 1);
        Mat out = tst.submat(adjusted);

        MatOfPoint refContour = getNormReference(ref);
        Rect refRect = Imgproc.boundingRect(refContour);

        Size orgSize = tst.size();

        Imgproc.resize(out, out, new Size(500, 500));

        Mat rotMat = Imgproc.getRotationMatrix2D(new Point((float)out.width()/2, (float)out.height()/2), r.angle, 1);
        Imgproc.warpAffine(out, out, rotMat, out.size());

        Mat toReturn = out.clone();
        Imgproc.cvtColor(out, out, Imgproc.COLOR_BGR2GRAY);

        Mat dst = Mat.zeros(out.size(), CvType.CV_32F);
        Mat dstNorm = new Mat();
        int threshold = 200;

        int blockSize = 2;
        int apertureSize = 3;
        double k = 0.04;
        Imgproc.cornerHarris(out, dst, blockSize, apertureSize, k);
        Core.normalize(dst, dstNorm, 0, 255, Core.NORM_MINMAX);
        float[] dstNormData = new float[(int) (dstNorm.total() * dstNorm.channels())];
        dstNorm.get(0, 0, dstNormData);
        ArrayList<Point> matchPoints = new ArrayList<>();
        for (int i = 0; i < dstNorm.rows(); i++) {
            for (int j = 0; j < dstNorm.cols(); j++) {
                if ((int) dstNormData[i * dstNorm.cols() + j] > threshold) {
                    //Imgproc.circle(toReturn, new Point(j, i), 5, new Scalar(0, 255, 0));
                    matchPoints.add(new Point(j, i));
                }
            }
        }

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(out, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        double maxVal = 0;
        int maxValIdx = 0;
        for (int contourIdx = 0; contourIdx < contours.size(); contourIdx++)
        {
            double contourArea = Imgproc.contourArea(contours.get(contourIdx));
            if (maxVal < contourArea)
            {
                maxVal = contourArea;
                maxValIdx = contourIdx;
            }
        }

        MatOfPoint match = contours.get(maxValIdx);

        MatOfPoint2f match2f = new MatOfPoint2f(match.toArray());
        MatOfPoint2f approxed = new MatOfPoint2f();
        Imgproc.approxPolyDP(match2f, approxed, 0.02 * Imgproc.arcLength(match2f, true), true);
        match = new MatOfPoint(approxed.toArray());

        //System.out.println(Imgproc.minAreaRect(new MatOfPoint2f(match.toArray())).angle);

        for(Point pt : match.toArray()){
            Imgproc.circle(toReturn, pt, 5, new Scalar(0, 255, 0));
        }

        MatOfPoint3f objPoints = new MatOfPoint3f();
        MatOfPoint2f imgPoints = new MatOfPoint2f();

        double epsilon = 5;
        Point[] refArr = refContour.toArray();
        int numMatches = 0;
        for(int i = 0; i < refArr.length; i += 4){
            Point p = refArr[i];

            //Imgproc.circle(toReturn, p, 5, new Scalar(0, 0, 255));
        }
        //Imgproc.resize(toReturn, toReturn, new Size(toReturn.width() * 4, toReturn.height() * 4));

        for(Point pt : match.toArray()){

            float sclX = (float)(((pt.x/(float)(adjusted.width + adjusted.x)) * 23.875) - (23.875/2));
            float sclY = (float)(((pt.y/(float)(adjusted.height + adjusted.y)) * 15.75) + (15.75/2));


            Point pt_rot = CvUtils.rotatePoint(pt, r.center, Math.toRadians(r.angle));
            Point pt_crop = new Point((pt_rot.x/500), (pt_rot.y/500));
            Point org_pt = new Point((pt_crop.x * adjusted.width) + adjusted.x, (pt_crop.y * adjusted.height) + adjusted.y);

            imgPoints.push_back(new MatOfPoint2f(org_pt));
            objPoints.push_back(new MatOfPoint3f(new Point3(sclX, sclY, 0)));
            Imgproc.circle(toReturn, pt, 5, new Scalar(255, 0, 255));
            numMatches ++;
        }

        MatOfDouble distCoeffs = new MatOfDouble();
        Mat intrinsic = new Mat(3, 3, CvType.CV_32FC1);
        Mat rvec = new Mat();
        Mat tvec = new Mat();

        double[] inrinsicFloat = new double[] {817.063304531327,0.0,325.9485286458284,0.0,819.4690054531818,236.2597899599986,0.0,0.0,1.0,0.0};
        intrinsic.put(0, 0, inrinsicFloat);

        double[] distFloat = new double[] {-0.014680796227423968,1.3720322590501144,-0.0028429009326778093,0.0010064951672061734,-5.347658630748131};
        distCoeffs.fromArray(distFloat);

        Mat axis = in.clone();

        if(numMatches >= 4) {
            Calib3d.solvePnPRansac(objPoints, imgPoints, intrinsic, distCoeffs, rvec, tvec);
            System.out.println(CvUtils.toPosition(tvec, rvec).dump());
            Calib3d.drawFrameAxes(axis, intrinsic, distCoeffs, rvec, tvec, 5, 3);
        }

        //System.out.println("Solved " + (System.currentTimeMillis() - start));

        return toReturn;
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

        hsv.release();

        return outMat;
    }

    public static Mat solvePnP(Mat in, RotatedRect rRect){
        Rect adjusted = CvUtils.enlargeROI(in, rRect.boundingRect(), 1);

        Mat tst = in.submat(adjusted);

        tst = BetterTowerGoalUtils.cropInRange(tst);

        Imgproc.cvtColor(tst, tst, Imgproc.COLOR_GRAY2BGR);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(10, 10));
        //Imgproc.morphologyEx(tst, tst, Imgproc.MORPH_CLOSE, kernel);

        Imgproc.resize(tst, tst, new Size(500, 500));

        Mat out = tst.clone();

        Mat rotMat = Imgproc.getRotationMatrix2D(new Point((float)out.width()/2, (float)out.height()/2), rRect.angle, 1);
        Imgproc.warpAffine(out, out, rotMat, out.size());

        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.cvtColor(out, out, Imgproc.COLOR_BGR2GRAY);
        Imgproc.findContours(out, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_NONE);
        Imgproc.cvtColor(out, out, Imgproc.COLOR_GRAY2BGR);

        MatOfPoint2f contour2f = new MatOfPoint2f(contours.get(0).toArray());

        MatOfPoint2f dp = new MatOfPoint2f();
        Imgproc.approxPolyDP(contour2f, dp, 0.01 * (Imgproc.arcLength(contour2f, true)), true);

        Imgproc.drawContours(out, Collections.singletonList(new MatOfPoint(dp.toArray())), -1, new Scalar(0, 0, 255), 3);

        return out;
    }
}
