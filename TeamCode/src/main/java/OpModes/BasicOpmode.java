package OpModes;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.util.RobotLog;

import Debug.Dashboard.Dashboard;
import Hardware.*;
import Hardware.Packets.*;
import State.EventSystem.*;
import State.*;
import MathUtils.*;

/**
 * Basic OpMode
 * All OpMode classes should extend this class
 * Contains setup for the statemachine, eventsystem, and hardware
 * Updates hardware by itself, and updates telemetry by itself
 */

public abstract class BasicOpmode extends LinearOpMode {
    public StateMachine stateMachine;
    public EventSystem eventSystem;
    public Hardware hardware;
    public OpmodeVariables opmodeVariables;
    protected double fps;
    private long prevTime;
    private SensorData sensorData;
    private HardwareData hardwareData;
    private boolean triggeredRun;

    public BasicOpmode(Hardware hardware){
        this.hardware = hardware;
        hardware.attachOpmode(this);
        stateMachine = new StateMachine();
        eventSystem = new EventSystem(stateMachine);
        triggeredRun = false;
        opmodeVariables = new OpmodeVariables();
    }

    @Override
    public void runOpMode() {
        sensorData = new SensorData();
        hardwareData = new HardwareData();
        setup();
        hardware.init();
        Thread robotThread = new Thread(hardware);
        eventSystem.triggerInit();
        prevTime = System.nanoTime();
        robotThread.start();
        while(!isStopRequested()){
            if(isStarted() && !triggeredRun){
                eventSystem.triggerStart();
                triggeredRun = true;
            }
            hardwareData = new HardwareData();
            //hardware.run();
            sensorData = hardware.getSensorData();
            if(MathUtils.nanoToDSec(System.nanoTime() - prevTime) != 0) {
                fps = 1 / MathUtils.nanoToDSec(System.nanoTime() - prevTime);
            }else{
                fps = 0;
            }
            prevTime = System.nanoTime();
            stateMachine.update(sensorData, hardwareData);
            eventSystem.update(sensorData, hardwareData);
            hardwareData.setDriveMotors(stateMachine.getDriveVelocities());
            hardwareData.setTimestamp(System.currentTimeMillis());
            hardware.addHardwarePacket(hardwareData);
            telemetry.update();
            Dashboard.sendTelemetry();
        }
        hardware.stop();
    }

    public abstract void setup();
}
