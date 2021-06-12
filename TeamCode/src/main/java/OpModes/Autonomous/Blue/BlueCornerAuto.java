package OpModes.Autonomous.Blue;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Hardware.*;
import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import MathSystems.Vector3;
import Odometry.ConstantVOdometer;
import OpModes.BasicOpmode;
import State.LogicState;

@Autonomous
public class BlueCornerAuto extends BasicOpmode {
    ConstantVOdometer odometer;
    Vector3 position, velocity;
    boolean doStarterStack = true;
    DELAY_LOCATION delayLocation = DELAY_LOCATION.FIRST_LOCATION;
    public BlueCornerAuto() {
        super(new UltimateGoalHardware());
    }

    @Override
    public void setup() {
        hardware.registerAll();
        hardware.enableAll();

        position = Vector3.ZERO();
        velocity = Vector3.ZERO();
        odometer = new ConstantVOdometer(stateMachine, position, velocity);

        eventSystem.onStart("Odometer", odometer);

        eventSystem.onInit("Setup", new LogicState(stateMachine) {
            int idx = 0;
            boolean pressedDown = false, pressedUp = false, pressedLeft = false, pressedRight = false;
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                telemetry.addLine("Setup");
                if(gamepad1.dpad_down && !pressedDown){
                    idx --;
                    if(idx < 0){
                        idx = 1;
                    }
                }
                if(gamepad1.dpad_up && !pressedUp){
                    idx ++;
                    if(idx > 1){
                        idx = 0;
                    }
                }

                if(idx == 0){
                    telemetry.addData("Do Starting Stack", "<" + doStarterStack + ">");
                    if((gamepad1.dpad_left && !pressedLeft) || (gamepad1.dpad_right && !pressedRight)){
                        doStarterStack = !doStarterStack;
                    }
                }else{
                    telemetry.addData("Do Starting Stack: ", doStarterStack);
                }

                if(idx == 1){
                    telemetry.addData("Wait Location", "<" + delayLocation + ">");
                    if(gamepad1.dpad_left && !pressedLeft){
                        delayLocation = delayLocation.previous();
                    }
                    if(gamepad1.dpad_right && !pressedRight){
                        delayLocation = delayLocation.next();
                    }
                }else{
                    telemetry.addData("Wait Location", delayLocation);
                }
            }
        });
    }

    enum DELAY_LOCATION{
        FIRST_LOCATION,
        SECOND_LOCATION,
        NO_DELAY,
        BOTH_LOCATIONS;

        private static DELAY_LOCATION[] elements = values();

        public static DELAY_LOCATION fromNumber(int number)
        {
            return elements[number % elements.length];
        }

        public DELAY_LOCATION next()
        {
            return elements[(this.ordinal() + 1) % elements.length];
        }

        public DELAY_LOCATION previous()
        {
            return elements[(this.ordinal() - 1) < 0 ? elements.length - 1 : this.ordinal() - 1];
        }
    }
}
