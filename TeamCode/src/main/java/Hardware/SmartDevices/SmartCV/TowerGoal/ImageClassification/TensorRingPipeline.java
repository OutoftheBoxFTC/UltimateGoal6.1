package Hardware.SmartDevices.SmartCV.TowerGoal.ImageClassification;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.tensorflow.lite.DataType;
import org.tensorflow.lite.support.image.TensorImage;
import org.tensorflow.lite.task.vision.detector.Detection;
import org.tensorflow.lite.task.vision.detector.ObjectDetector;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import Hardware.SmartDevices.SmartCV.TowerGoal.TFLite.Detector;
import Hardware.SmartDevices.SmartCV.TowerGoal.TFLite.TFLiteObjectDetectionAPIModel;
import Hardware.SmartDevices.SmartCV.TowerGoal.TensorPipeline;

public class TensorRingPipeline extends OpenCvPipeline {
    private Detector objectDetector;

    public TensorRingPipeline(HardwareMap map){

        try {
            objectDetector = TFLiteObjectDetectionAPIModel.create(map.appContext, "ring.tflite", 300, false, 10,"Zero", "One", "Four", "Null");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat resized = new Mat();
        Imgproc.resize(input, resized, new Size(300, 300));
        Bitmap bmp = Bitmap.createBitmap(resized.cols(), resized.rows(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(resized, bmp);

        List<Detector.Recognition> dets = objectDetector.recognizeImage(bmp);

        for(Detector.Recognition d : dets){
            Imgproc.putText(input, d.toString(), new Point(input.width()/2.0, input.height()/2.0), Imgproc.FONT_HERSHEY_COMPLEX, 1.0, new Scalar(0, 0, 255));
            Rect r = new Rect(
                    new Point((d.getLocation().right / 300) * input.width(), (d.getLocation().top / 300) * input.height()),
                    new Point((d.getLocation().left / 300) * input.width(), (d.getLocation().bottom / 300) * input.height())
            );
            Imgproc.rectangle(input, r, new Scalar(255, 0, 0));
        }

        return input;
    }

    public List<Classifier.Recognition> getMasterRecogs() {
        return new ArrayList<>();
    }
}
