package OpModes.Debug;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Hardware.*;
import OpModes.BasicOpmode;
@TeleOp
public class DashboardTests extends BasicOpmode {
    public DashboardTests() {
        super(new TestHardware());
    }

    @Override
    public void setup() {

    }
}
