package OpModes.Debug;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.internal.WebInterface.InterfaceHandler;
import org.firstinspires.ftc.robotcontroller.internal.WebInterface.VariableConfig.Configurable;

import Hardware.*;
import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import MathSystems.Vector3;
import OpModes.BasicOpmode;
import State.LogicState;

@Configurable
@TeleOp
public class ConfiruationTest extends BasicOpmode {
    public static double dTestVar = 0.5;
    public static boolean bTestVar = true;
    public static String strTestVar = "foo";
    public static Vector3 testVector = Vector3.ZERO();

    public ConfiruationTest() {
        super(new TestHardware());
    }

    @Override
    public void setup() {
        eventSystem.onInit("Init", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                InterfaceHandler.getInstance().internalAddTelemetry("TestDouble", dTestVar);
                InterfaceHandler.getInstance().internalAddTelemetry("TestBool", bTestVar);
                InterfaceHandler.getInstance().internalAddTelemetry("TestString", strTestVar);
                InterfaceHandler.getInstance().internalUpdateTelemetry();
            }
        });
    }
}
