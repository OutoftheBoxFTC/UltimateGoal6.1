package org.firstinspires.ftc.robotcontroller.internal.WebInterface.JsonPackets;
public class TelemetryPacket{
	int opcode = 1;
	public String telemetry = "null";
	public String position;
	public long timestamp;
}