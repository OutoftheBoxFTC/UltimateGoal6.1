package org.firstinspires.ftc.robotcontroller.internal.WebInterface;

import com.qualcomm.robotcore.util.RobotLog;

import org.java_websocket.WebSocket;
import org.java_websocket.handshake.ClientHandshake;
import org.java_websocket.server.WebSocketServer;

import java.net.InetSocketAddress;
import java.util.ArrayList;

public class VideoStreamSocket extends WebSocketServer {
    private ArrayList<WebSocket> sockets;
    public VideoStreamSocket(int port) {
        super(new InetSocketAddress(port));
        sockets = new ArrayList<>();
    }

    @Override
    public void onClose(WebSocket conn, int code, String reason, boolean remote) {
        sockets.remove(conn);
        RobotLog.i("Connection to " + conn.getRemoteSocketAddress().getAddress().getHostAddress() + " closed with code " + code);
    }

    @Override
    public void onError(WebSocket conn, Exception excep) {
        if(conn != null)
            RobotLog.i(conn.getRemoteSocketAddress().getAddress().getHostAddress() + " reported error " + excep.toString());
    }

    @Override
    public void onMessage(WebSocket conn, String message) {

    }

    @Override
    public void onOpen(WebSocket conn, ClientHandshake handshake) {
        sockets.add(conn);
        String str = "" + System.currentTimeMillis() + Math.round(Math.random() * 1000);
        RobotLog.i("New connection from " + conn.getRemoteSocketAddress().getAddress().getHostAddress());
    }

    public void sendAll(String message) {
        for(WebSocket socket : sockets) {
            if(socket != null)
                socket.send(message);
        }
    }

    @Override
    public void onStart() {

    }
}
