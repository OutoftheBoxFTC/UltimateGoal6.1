package Hardware.SmartDevices.SmartCV.TowerGoal.ImageClassification;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class TensorRingPipeline extends OpenCvPipeline {
    private Classifier classifier;
    private final List<Classifier.Recognition> masterRecogs;

    public TensorRingPipeline(HardwareMap map){

        try {
            classifier = Classifier.create(map.appContext, Classifier.Model.QUANTIZED_EFFICIENTNET, Classifier.Device.GPU, 2, "Zero", "One", "Four", "Null");
        } catch (IOException e) {
            e.printStackTrace();
        }
        masterRecogs = new ArrayList<>();
    }

    @Override
    public Mat processFrame(Mat input) {
        Bitmap bmp = Bitmap.createBitmap(input.cols(), input.rows(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(input, bmp);

        List<Classifier.Recognition> recogs = classifier.recognizeImage(bmp, 0);
        synchronized (masterRecogs){
            masterRecogs.clear();
            masterRecogs.addAll(recogs);
        }

        return input;
    }

    public List<Classifier.Recognition> getMasterRecogs() {
        return masterRecogs;
    }
}
