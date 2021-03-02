package Hardware.CustomClasses;

import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;

public class VuforiaPlus extends VuforiaLocalizerImpl {
    public VuforiaPlus(Parameters parameters) {
        super(parameters);
    }

    public void stop(){
        super.deinitTracker();
        super.destroyTrackables();
        super.stopTracker();
        super.stopCamera();
        super.stopAR();
        super.close();
    }
}
