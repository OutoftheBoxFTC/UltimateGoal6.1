package org.firstinspires.ftc.robotcontroller.internal.WebInterface;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.util.Log;

import com.google.gson.Gson;
import com.qualcomm.robotcore.eventloop.EventLoop;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcontroller.internal.WebInterface.JsonPackets.FileEditPacket;
import org.firstinspires.ftc.robotcontroller.internal.WebInterface.JsonPackets.OpmodePacket;
import org.firstinspires.ftc.robotcontroller.internal.WebInterface.JsonPackets.TelemetryPacket;
import org.firstinspires.ftc.robotcontroller.internal.WebInterface.JsonPackets.VariableEditPacket;
import org.firstinspires.ftc.robotcontroller.internal.WebInterface.JsonPackets.VariablesPacket;
import org.firstinspires.ftc.robotcontroller.internal.WebInterface.VSD.VirtualSD;
import org.firstinspires.ftc.robotcontroller.internal.WebInterface.VariableConfig.ClassHandler;
import org.firstinspires.ftc.robotcontroller.internal.WebInterface.VariableConfig.UpdateableVariable;
import org.firstinspires.ftc.robotcontroller.internal.WebInterface.VariableConfig.VariableType;
import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.robotcore.internal.opmode.RegisteredOpModes;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketException;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashSet;
import java.util.Objects;
import java.util.Set;

public class InterfaceHandler implements Runnable, OpModeManagerImpl.Notifications{
    private static final int WEBSOCKET_PORT = 9000;
    private static final int VIDEO_PORT = 8500;

    private static final String TAG = "InterfaceHandler";

    private static InterfaceHandler instance = new InterfaceHandler();
    private EventLoop eventLoop;
    private OpModeManagerImpl opModeManager;
    private ArrayList<String> opmodes, consoleCommands, telemetryPacket, telemetryStrings, logText, colors;
    private TelemetryPacket packet;
    private String ping;
    private Thread mainThread;
    private JavaHTTPServer javaHTTPServer;
    private ArrayList<String> packets;
    private HashMap<String, String> telemetry, editableValues;
    private volatile boolean running, sendTelemetry;
    public static InterfaceHandler getInstance(){
        return instance;
    }
    private Context context;
    private long telemetryTimer;
    private String position;
    private Gamepad vGamepad;
    private String mousePos;
    private OpMode activeOpMode;
    private ArrayList<ArrayList<Bitmap>> cameras;

    public InterfaceHandler(){
        packet = new TelemetryPacket();
        opmodes = new ArrayList<>();
        packets = new ArrayList<>();
        telemetry = new HashMap<>();
        editableValues = new HashMap<>();
        consoleCommands = new ArrayList<>();
        telemetryStrings = new ArrayList<>();
        packets.add("");
        consoleCommands.add("");
        //running = true;
        sendTelemetry = false;
        telemetryPacket = new ArrayList<>();
        running = true;
        position = "0, 0, 0";
        mousePos = "0, 0";
        telemetryTimer = 0;
        vGamepad = new Gamepad();
        cameras = new ArrayList<>();
        logText = new ArrayList<>();
        colors = new ArrayList<>();
        new Thread(this).start();
    }

    public void start(){
        Thread serverThread = new Thread(new Runnable() {
            @Override
            public void run() {
                JavaHTTPServer.start();
            }
        });
        serverThread.start();
    }

    public void addContext(Context context){
        this.context = context;
        VirtualSD.getInstance().addContext(context);
    }

    public static void attachEventLoop(EventLoop eventLoop){
        instance.internalAttachLoop(eventLoop);
    }

    public void internalAttachLoop(EventLoop eventLoop){
        this.eventLoop = eventLoop;
        this.opModeManager = eventLoop.getOpModeManager();
        opModeManager.registerListener(this);
        new Thread(new SyncOpmodes()).start();
    }

    @Override
    public void run(){
        WebsocketHandler handler = new WebsocketHandler(WEBSOCKET_PORT);
        handler.setReuseAddr(true);
        handler.start();
        RobotLog.i("Dashboard Started");
        Gson gson = SimpleGson.getInstance();
        long timer = 0;
        while(running){
            timer = System.currentTimeMillis() + 100;
            HashMap<String, String> newMessages = handler.getNewMessages();
            for(String s : newMessages.keySet()){
                //RobotLog.ii("Request Incoming", newMessages.get(s));
                InterfaceHandler.log("Request " + newMessages.get(s));
                if(Objects.equals(newMessages.get(s), "OPMODES")){
                    OpmodePacket packet = new OpmodePacket();
                    packet.opmodes = opmodes.toArray(new String[0]);
                    handler.send(s, gson.toJson(packet));
                }
                if(newMessages.get(s).startsWith("INIT")){
                    opModeManager.initActiveOpMode(newMessages.get(s).replace("INIT ", ""));
                }
                if(newMessages.get(s).equals("START")){
                    opModeManager.startActiveOpMode();
                }
                if(newMessages.get(s).equals("STOP")){
                    OpMode opName = opModeManager.getActiveOpMode();
                    opModeManager.requestOpModeStop(opName);
                }
                if(newMessages.get(s).startsWith("GAMEPAD")){
                    setGamepad(newMessages.get(s).replace("GAMEPAD ", ""));
                }
                if(newMessages.get(s).startsWith("MOUSECOORDS")){
                    setMousePos(newMessages.get(s).replace("MOUSECOORDS ", ""));
                }
                if(newMessages.get(s).startsWith("DELETEFILE")){
                    String str = newMessages.get(s).replace("DELETEFILE", "");
                    try {
                        VirtualSD.getInstance().getFile(str).deleteFile();
                    } catch (IOException e) {
                        error(e.getMessage());
                    }
                }
                if(newMessages.get(s).startsWith("CREATEFOLDER")){
                    String str = newMessages.get(s).replace("CREATEFOLDER", "");
                    try {
                        VirtualSD.getInstance().addFolder(str);
                    } catch (IOException e) {
                        error(e.getMessage());
                    }
                }
                if(newMessages.get(s).startsWith("DELETEFOLDER")){
                    String str = newMessages.get(s).replace("DELETEFOLDER", "");
                    VirtualSD.getInstance().deleteFolder(str);
                }
            }
            if(sendTelemetry){
                packet.timestamp = System.currentTimeMillis();
                packet.console = logText.toArray(new String[0]);
                packet.colors = colors.toArray(new String[0]);
                synchronized (telemetry) {
                    for (String s : telemetry.keySet()) {
                        telemetryPacket.add(s + " : " + telemetry.get(s));
                    }
                }
                telemetryPacket.addAll(telemetryStrings);
                packet.telemetry = telemetryPacket.toArray(new String[0]);
                sendTelemetry = false;
                packet.position = position;
                telemetryPacket.clear();
                synchronized (telemetry) {
                    telemetry.clear();
                }
                handler.sendAll(gson.toJson(packet));
            }else{
                packet.timestamp = System.currentTimeMillis();
                packet.console = logText.toArray(new String[0]);
                packet.colors = colors.toArray(new String[0]);
                if(opModeManager != null)
                    packet.selectedOpmode = opModeManager.getActiveOpModeName();
                handler.sendAll(gson.toJson(packet));
            }

            while(System.currentTimeMillis() < timer); //Limit to 100 ms per cycle so we don't accidentally DOS the client
        }
        try {
            handler.stop();
        } catch (IOException e) {
            e.printStackTrace();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void onOpModePreInit(OpMode opMode) {
        packet.init = true;
        packet.start = false;
        this.activeOpMode = opMode;
    }

    @Override
    public void onOpModePreStart(OpMode opMode) {
        packet.init = false;
        packet.start = true;
        this.activeOpMode = opMode;
    }

    @Override
    public void onOpModePostStop(OpMode opMode) {
        packet.init = true;
        packet.start = true;
        this.activeOpMode = opMode;
    }

    private class SyncOpmodes implements Runnable {
        @Override
        public void run() {
            RobotLog.i("Opmodes Started");
            RegisteredOpModes.getInstance().waitOpModesRegistered();
            RobotLog.i("Opmodes init");
            for(OpModeMeta opModeMeta : RegisteredOpModes.getInstance().getOpModes()){
                opmodes.add(opModeMeta.name);
                RobotLog.ii("OpmodeName", opModeMeta.name);
            }
            Set<String> removeDupes = new LinkedHashSet<String>(opmodes);
            opmodes.clear();
            opmodes.addAll(removeDupes);
            Collections.sort(opmodes);
        }
    }

    public void internalAddTelemetry(String header, Object data){
        synchronized (telemetry) {
            telemetry.put(header, data.toString());
        }
    }

    public void internalAddLine(String line){
        telemetryStrings.add(line);
    }

    public ArrayList<String> getTelemetry(){
        return telemetryPacket;
    }

    public ArrayList<String> getOpmodes(){
        return opmodes;
    }

    public String getConsoleCommand(){
        if(!consoleCommands.get(0).equals("")){
            String s = consoleCommands.get(0);
            consoleCommands.remove(0);
            return s;
        }
        return "";
    }

    public InputStream getInputStream(String name){
        String s = name.replaceFirst("/", "");
        if(name.contains("robot") || name.contains("field")){
            Log.i("Dashboard", name);
        }
        try {
            return context.getAssets().open(s);
        } catch (IOException e) {
            try {
                return context.getAssets().open("index.html");
            } catch (IOException ex) {
                ex.printStackTrace();
            }
        }
        return null;
    }

    public void internalAddCameras(Bitmap... bmps){
        ArrayList<Bitmap> arr = new ArrayList<>();
        for(Bitmap b : bmps){
            arr.add(b);
        }
        cameras.add(arr);
    }

    public void addPacket(String s){
        packets.add(s);
    }

    public void internalUpdateTelemetry(){
        sendTelemetry = true;
        telemetryTimer = System.currentTimeMillis() + 1000;
    }

    public void internalSetPosition(String pos){
        this.position = pos;
    }

    public String getPosition(){
        return this.position;
    }

    public void setGamepad(String s){
        String[] arr = s.split(",");
        if(arr.length >= 15) {
            vGamepad.a = arr[0].equals("1");
            vGamepad.b = arr[1].equals("1");
            vGamepad.x = arr[2].equals("1");
            vGamepad.y = arr[3].equals("1");
            vGamepad.left_bumper = arr[4].equals("1");
            vGamepad.right_bumper = arr[5].equals("1");
            vGamepad.left_trigger = Float.parseFloat(arr[6]);
            vGamepad.right_trigger = Float.parseFloat(arr[7]);
            vGamepad.back = arr[8].equals("1");
            vGamepad.start = arr[9].equals("1");
            vGamepad.left_stick_button = arr[10].equals("1");
            vGamepad.right_stick_button = arr[11].equals("1");
            vGamepad.dpad_up = arr[12].equals("1");
            vGamepad.dpad_down = arr[13].equals("1");
            vGamepad.dpad_left = arr[14].equals("1");
            vGamepad.dpad_right = arr[15].equals("1");
            vGamepad.left_stick_x = Float.parseFloat(arr[17]);
            vGamepad.left_stick_y = Float.parseFloat(arr[18]);
            vGamepad.right_stick_x = Float.parseFloat(arr[19]);
            vGamepad.right_stick_y = Float.parseFloat(arr[20]);
        }
    }

    public Gamepad getVirtualGamepad(){
        return vGamepad;
    }

    public void setMousePos(String s){
        this.mousePos = s;
    }

    public String getMousePos(){
        return mousePos;
    }

    public void stop(){
        running = false;
    }

    public static Bitmap combineImagesHoriz(Bitmap c, Bitmap s) {
        Bitmap cs = null;

        int width, height = 0;

        if(c.getWidth() > s.getWidth()) {
            width = c.getWidth() + s.getWidth();
            height = c.getHeight();
        } else {
            width = s.getWidth() + s.getWidth();
            height = c.getHeight();
        }

        cs = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);

        Canvas comboImage = new Canvas(cs);

        comboImage.drawBitmap(c, 0f, 0f, null);
        comboImage.drawBitmap(s, c.getWidth(), 0f, null);
        return cs;
    }

    public static void log(String message){
        instance.colors.add("Gray");
        instance.logText.add(System.currentTimeMillis() + ": " + message);
    }

    public static void warn(String message){
        instance.colors.add("Orange");
        instance.logText.add(System.currentTimeMillis() + ": " + message);
    }

    public static void error(String message){
        instance.colors.add("Red");
        instance.logText.add(System.currentTimeMillis() + ": " + message);
    }
}
