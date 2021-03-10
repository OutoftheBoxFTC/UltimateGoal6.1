package Hardware.SmartDevices.SmartCV;

import android.content.Context;

import org.firstinspires.ftc.teamcode.R;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.IOException;

import Hardware.SmartDevices.SmartCV.Utils.BetterTowerGoalUtils;
import Hardware.SmartDevices.SmartCV.Utils.CvUtils;

public class TowergoalPipeline extends OpenCvPipeline {
    private Mat cropCopy;
    private MatOfPoint refContour;
    private Rect boundingRect;

    public TowergoalPipeline(Context context) throws IOException {
        boundingRect = new Rect(0, 0, 0, 0);
        cropCopy = new Mat();
        Mat refMat = Utils.loadResource(context, R.raw.ref);

        refContour = BetterTowerGoalUtils.getReference(refMat);
        refMat.release();
    }

    @Override
    public Mat processFrame(Mat input) {
        MatOfPoint m = BetterTowerGoalUtils.getMatchRect(refContour, input);
        boundingRect = Imgproc.boundingRect(m);

        input.copyTo(cropCopy);
        Imgproc.rectangle(cropCopy, boundingRect, new Scalar(0, 255, 0), 5);

        m.release();

        return cropCopy;
    }

    public Rect getBoundingRect() {
        return boundingRect;
    }
}
