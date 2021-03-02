package org.firstinspires.ftc.robotcontroller.internal.WebInterface;

import com.qualcomm.robotcore.util.RobotLog;

import org.java_websocket.WebSocket;
import org.java_websocket.handshake.ClientHandshake;
import org.java_websocket.server.WebSocketServer;

import java.net.InetSocketAddress;
import java.util.ArrayList;
import java.util.HashMap;

public class WebsocketHandler extends WebSocketServer {
	private ArrayList<WebSocket> sockets;
	private HashMap<String, String> incomingMessages;
	private HashMap<WebSocket, String> ids;
	public WebsocketHandler(int port) {
        super(new InetSocketAddress(port));
        incomingMessages = new HashMap<>();
        sockets = new ArrayList<>();
        ids = new HashMap<>();
	}

	@Override
	public void onClose(WebSocket conn, int code, String reason, boolean remote) {
		sockets.remove(conn);
		ids.remove(conn);
		RobotLog.i("Connection to " + conn.getRemoteSocketAddress().getAddress().getHostAddress() + " closed with code " + code);
	}

	@Override
	public void onError(WebSocket conn, Exception excep) {
		if(conn != null)
			RobotLog.i(conn.getRemoteSocketAddress().getAddress().getHostAddress() + " reported error " + excep.toString());
	}

	@Override
	public void onMessage(WebSocket conn, String message) {
		synchronized(incomingMessages) {
			incomingMessages.put(ids.get(conn), message);
		}
	}

	@Override
	public void onOpen(WebSocket conn, ClientHandshake handshake) {
		sockets.add(conn);
		String str = "" + System.currentTimeMillis() + Math.round(Math.random() * 1000);
		ids.put(conn, str);
		RobotLog.i("New connection from " + conn.getRemoteSocketAddress().getAddress().getHostAddress());
	}

	@Override
	public void onStart() {

	}
	
	public ArrayList<WebSocket> getConnectedSockets(){
		ArrayList<WebSocket> socketClone = new ArrayList<>();
		socketClone.addAll(sockets);
		return socketClone;
	}
	
	public void send(String id, String message) {
		for(WebSocket socket : ids.keySet()) {
			if(ids.get(socket).equals(id)) {
				socket.send(message);
			}
		}
	}
	
	public void sendAll(String message) {
		for(WebSocket socket : sockets) {
			if(socket != null)
				socket.send(message);
		}
	}
	
	public HashMap<String, String> getNewMessages(){
		HashMap<String, String> messages = new HashMap<>();
		synchronized(incomingMessages) {
			messages.putAll(incomingMessages);
			incomingMessages.clear();
		}
		return messages;
	}

}
