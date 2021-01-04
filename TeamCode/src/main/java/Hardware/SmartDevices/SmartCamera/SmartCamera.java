package Hardware.SmartDevices.SmartCamera;

import android.graphics.Bitmap;
import android.graphics.ImageFormat;
import android.os.Handler;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureRequest;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSequenceId;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraException;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.internal.collections.EvictingBlockingQueue;
import org.firstinspires.ftc.robotcore.internal.network.CallbackLooper;
import org.firstinspires.ftc.robotcore.internal.system.ContinuationSynchronizer;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.ArrayBlockingQueue;

import Hardware.SmartDevices.SmartDevice;

public class SmartCamera extends SmartDevice {
    private static int IMAGE_FORMAT = ImageFormat.YUY2;
    private Camera camera;
    private int fps;
    private Size size;
    private ContinuationSynchronizer<CameraCaptureSession> synchronizer;
    private EvictingBlockingQueue<Bitmap> frameQueue;
    private SmartCameraConfiguration configuration;

    public SmartCamera(CameraName name, SmartCameraConfiguration configuration){
        this.configuration = configuration;
        Deadline deadLine = new Deadline(configuration.registerTimeout, configuration.timeoutUnit);
        camera = ClassFactory.getInstance().getCameraManager().requestPermissionAndOpenCamera(deadLine, name, null);
        final Handler handler = CallbackLooper.getDefault().getHandler();
        CameraCharacteristics cameraCharacteristics = name.getCameraCharacteristics();
        size = cameraCharacteristics.getDefaultSize(IMAGE_FORMAT);
        fps = cameraCharacteristics.getMaxFramesPerSecond(IMAGE_FORMAT, size);

        frameQueue = new EvictingBlockingQueue<Bitmap>(new ArrayBlockingQueue<Bitmap>(2));
        frameQueue.setEvictAction(new Consumer<Bitmap>() {
            @Override public void accept(Bitmap frame) {
                frame.recycle();
            }
        });

        synchronizer = new ContinuationSynchronizer<>();

        try {
            camera.createCaptureSession(Continuation.create(handler, new CameraCaptureSession.StateCallbackDefault(){
                @Override
                public void onConfigured(@NonNull CameraCaptureSession session) {
                    try {
                        CameraCaptureRequest request = camera.createCaptureRequest(IMAGE_FORMAT, size, fps);
                        session.startCapture(request,
                                new CameraCaptureSession.CaptureCallback() {
                                    @Override public void onNewFrame(@NonNull CameraCaptureSession session, @NonNull CameraCaptureRequest request, @NonNull CameraFrame cameraFrame) {
                                        /** A new frame is available. The frame data has <em>not</em> been copied for us, and we can only access it
                                         * for the duration of the callback. So we copy here manually. */
                                        Bitmap bmp = request.createEmptyBitmap();
                                        cameraFrame.copyToBitmap(bmp);
                                        frameQueue.offer(bmp);
                                    }
                                },
                                Continuation.create(handler, new CameraCaptureSession.StatusCallback() {
                                    @Override public void onCaptureSequenceCompleted(@NonNull CameraCaptureSession session, CameraCaptureSequenceId cameraCaptureSequenceId, long lastFrameNumber) {
                                    }
                                })
                        );
                        synchronizer.finish(session);
                    } catch (CameraException e) {
                        e.printStackTrace();
                    }
                }
            }));
        } catch (CameraException e) {
            e.printStackTrace();
        }
    }

    public Bitmap getFrame(){
        return frameQueue.poll();
    }

    @Override
    public void calibrate() {
        
    }

    @Override
    public void update() {

    }
}
