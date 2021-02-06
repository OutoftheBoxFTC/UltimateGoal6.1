package OpModes.Debug;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Hardware.*;
import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import OpModes.BasicOpmode;
import State.LogicState;
@TeleOp
public class ShooterLoadTiming extends BasicOpmode {
    long timeCoeff = 75;
    public ShooterLoadTiming() {
        super(new UltimateGoalHardware());
    }

    @Override
    public void setup() {
        hardware.registerAll();
        hardware.enableAll();
        eventSystem.onStart("Load Arm", new LogicState(stateMachine) {
            int state = 0;
            long timer = 0;
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                if(state == 0){
                    hardwareData.setShooterLoadArm(0.5);
                    timer = System.currentTimeMillis() + timeCoeff;
                    state = 1;
                }
                if(state == 1){
                    if(System.currentTimeMillis() >= timer){
                        state = 2;
                    }
                }
                if(state == 2){
                    hardwareData.setShooterLoadArm(0.3);
                    timer = System.currentTimeMillis() + timeCoeff;
                    state = 3;
                }
                if(state == 3){
                    if(System.currentTimeMillis() >= timer){
                        state = 0;
                    }
                }
            }
        });

        eventSystem.onStart("Adjuster", new LogicState(stateMachine) {
            boolean prevDown = false, prevUp = false;
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                if(!prevDown && gamepad1.dpad_down){
                    timeCoeff -= 1;
                }
                if(!prevUp && gamepad1.dpad_up){
                    timeCoeff += 1;
                }
                prevDown = gamepad1.dpad_down;
                prevUp = gamepad1.dpad_up;
            }
        });
    }
}
