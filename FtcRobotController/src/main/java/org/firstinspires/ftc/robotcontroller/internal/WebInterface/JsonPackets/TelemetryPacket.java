package org.firstinspires.ftc.robotcontroller.internal.WebInterface.JsonPackets;
public class TelemetryPacket{
	int opcode = 1;
	public String[] telemetry = new String[0];
	public String position = "0,0,0";
	public String[] console;
	public String[] colors;
	public String selectedOpmode = "";
	public long timestamp;
	public volatile boolean init = false;
	public volatile boolean start = false;
}