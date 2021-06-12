package org.firstinspires.ftc.robotcontroller.internal.WebInterface;


import android.util.Base64;
import android.util.Log;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcontroller.internal.WebInterface.JsonPackets.FileIndexPacket;
import org.firstinspires.ftc.robotcontroller.internal.WebInterface.JsonPackets.OpmodePacket;
import org.firstinspires.ftc.robotcontroller.internal.WebInterface.VSD.VirtualSD;
import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;

import java.io.BufferedOutputStream;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.PrintWriter;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.URLDecoder;
import java.net.URLEncoder;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Date;
import java.util.List;
import java.util.StringTokenizer;

public class JavaHTTPServer implements Runnable {

	static final File WEB_ROOT = new File("");
	static final String DEFAULT_FILE = "index.html";
	static final String FILE_NOT_FOUND = "r404.html";
	static final String METHOD_NOT_SUPPORTED = "not_supported.html";
	// port to listen connection
	static final int PORT = 4750;

	// verbose mode
	static final boolean verbose = false;

	// Client Connection via Socket Class
	private Socket connect;

	public JavaHTTPServer(Socket c){
		connect = c;
	}

	public static void start(){
		try {
			ServerSocket serverConnect = new ServerSocket(PORT);
			Log.i("Dashboard: ", "Server started.\nListening for connections on port : " + PORT + " ...\n");

			// we listen until user halts server execution
			while (true) {
				JavaHTTPServer myServer = new JavaHTTPServer(serverConnect.accept());

				if (verbose) {
					Log.i("Dashboard", "Connecton opened. (" + new Date() + ")");
				}

				// create dedicated thread to manage the client connection
				Thread thread = new Thread(myServer);
				thread.start();
			}

		} catch (IOException e) {
			Log.i("Dashboard", "Server Connection error : " + e.getMessage());
		}
	}

	@Override
	public void run() {
		// we manage our particular client connection
		BufferedReader in = null; PrintWriter out = null; BufferedOutputStream dataOut = null;
		String fileRequested = null;
		try {
			// we read characters from the client via input stream on the socket
			in = new BufferedReader(new InputStreamReader(connect.getInputStream(), "UTF8"));
			// we get character output stream to client (for headers)
			out = new PrintWriter(connect.getOutputStream());
			// get binary output stream to client (for requested data)
			dataOut = new BufferedOutputStream(connect.getOutputStream());

			// get first line of the request from the client
			String input = in.readLine();
			// we parse the request with a string tokenizer
			if((input == null)){
				return;
			}
			StringTokenizer parse = new StringTokenizer(input);
			String method = parse.nextToken().toUpperCase(); // we get the HTTP method of the client
			// we get file requested

			fileRequested = parse.nextToken().toLowerCase();
			String test = "", test2 = "";
			//char[] tmp = new char[1024];
			//in.read(tmp);
			//String buffer = new String(tmp);

			// we support only GET, HEAD, POST, and OPTIONS methods, we check
			if (!method.equals("GET")  &&  !method.equals("HEAD") && !method.equals("POST") && !method.equals("OPTIONS")) {
				if (verbose) {
					Log.i("Dashboard", "501 Not Implemented : " + method + " method.");
				}

				// we return the not supported file to the client
				File file = new File(WEB_ROOT, METHOD_NOT_SUPPORTED);
				String contentMimeType = "text/html";
				//read content to return to client
				byte[] fileData = readFileData(fileRequested);

				// we send HTTP Headers with data to client
				out.println("HTTP/1.1 501 Not Implemented");
				out.println("Server: Java HTTP Server from FTC 7244 : 1.0");
				out.println("Date: " + new Date());
				out.println("Content-type: " + contentMimeType);
				out.println(); // blank line between headers and content, very important !
				out.flush(); // flush character output stream buffer
				// file
				dataOut.write(fileData, 0, fileData.length);
				dataOut.flush();

			} else if(method.equals("GET") || method.equals("HEAD")){
				// GET or HEAD method
				if (fileRequested.endsWith("/")) {
					fileRequested += DEFAULT_FILE;
				}

				//File file = new File(WEB_ROOT, fileRequested);
				//int fileLength = (int) file.length();
				String content = getContentType(fileRequested);

				if (method.equals("GET")) { // GET method so we return content
					byte[] fileData = new byte[128];
					fileData = readFileData(fileRequested);

					// send HTTP Headers
					out.println("HTTP/1.1 200 OK");
					out.println("Server: Java HTTP Server from FTC 7244 : 1.0");
					out.println("Date: " + new Date());
					out.println("Content-type: " + content);
					out.println("Access-Control-Allow-Origin: " + "*");
					//out.println("Content-length: " + fileLength);
					out.println(); // blank line between headers and content, very important !
					out.flush(); // flush character output stream buffer

					dataOut.write(fileData, 0, fileData.length);
					dataOut.flush();
				}

				if (verbose) {
					Log.i("Dashboard", "File " + fileRequested + " of type " + content + " returned");
				}

			}else if(method.equals("OPTIONS")){
				out.println("HTTP/1.1 200 OK");
				out.println("Server: Java HTTP Server from FTC 7244 : 1.0");
				out.println("Date: " + new Date());
				out.println("Content-type: " + "text/plain");
				out.println("Access-Control-Allow-Origin: " + "*");
				out.println("Access-Control-Allow-Methods: " + "*");
				out.println("Access-Control-Allow-Headers: " + "*");
				out.println();
				out.flush();
			}
			else {
                String buffer = "";
                //Content-Length
				char[] cTmp = new char[1024];
				in.read(cTmp);
				out.println("HTTP/1.1 200 OK");
				out.println("Server: Java HTTP Server from FTC 7244 : 1.0");
				out.println("Date: " + new Date());
				out.println("Content-type: " + "text/plain");
				out.println("Access-Control-Allow-Origin: " + "*");
				out.println();
				out.flush();
				String[] arrT = new String(cTmp).split("\n");
				int len = 0;
				for(String s : arrT){
					if(s.contains("Content-Length")){
						len = Integer.valueOf(s.split(": ")[1].replaceAll("\uFEFF", "").trim());
					}
				}
				String tmp = arrT[arrT.length-1].substring(0, len);
                buffer = tmp;
                buffer = buffer.split("ENDTRANSMISSION")[0];
				String[] arr = URLDecoder.decode(buffer, "ASCII").split("\n");
				String string = URLDecoder.decode(buffer, "ASCII");
				if(fileRequested.toLowerCase().contains("upload")){
					InterfaceHandler.getInstance().addPacket(string);
				}else {
					if(fileRequested.endsWith(".vfile")) {
						fileRequested = fileRequested.replace(".vfile", "");
						//VirtualSD.getInstance().getFile(fileRequested).clear();
						//VirtualSD.getInstance().getFile(fileRequested).write(string);
					}else if (fileRequested.endsWith(".vfolder")){
						fileRequested = fileRequested.replace(".vfolder", "");
						VirtualSD.getInstance().addFolder(fileRequested);
					}else if(fileRequested.endsWith(".vdeletefile")){
						fileRequested = fileRequested.replace(".vdeletefile", "");
						VirtualSD.getInstance().getFile(fileRequested).deleteFile();
					}else if(fileRequested.endsWith(".vdeletefolder")){
						fileRequested = fileRequested.replace(".vdeletefolder", "");
						VirtualSD.getInstance().deleteFolder(fileRequested);
					}
				}
			}

		} catch (FileNotFoundException fnfe) {
			try {
				fileNotFound(out, dataOut, fileRequested);
			} catch (IOException ioe) {
				Log.i("Dashboard", "Error with file not found exception : " + ioe.getMessage());
			}

		} catch (IOException ioe) {
			Log.i("Dashboard", "Server error : " + ioe);
		} finally {
			try {
				in.close();
				out.close();
				dataOut.close();
				connect.close(); // we close socket connection
			} catch (Exception e) {
				Log.i("Dashboard", "Error closing stream : " + e.getMessage());
			}

			if (verbose) {
				Log.i("Dashboard", "Connection closed.\n");
			}
		}


	}

	private byte[] readFileData(String str) throws IOException {
		if(verbose){
			RobotLog.ii("DashboardFileRecord", str);
		}
		if(str.equals("/gateway")){
			String ip;
			ip = getIPAddress();
			return ("ws://" + ip + ":9000/").getBytes("UTF-8");
		}
		if(str.equals("/stream")){
			String ip;
			ip = getIPAddress();
			return ("ws://" + ip + ":8500/").getBytes("UTF-8");
		}
		if(str.equals("/ping.txt")){
			return String.valueOf(System.currentTimeMillis()).getBytes("UTF8");
		}
		if(str.equals("/telemetry.txt")){
			StringBuilder s = new StringBuilder();
			for(String key : InterfaceHandler.getInstance().getTelemetry()){
				s.append(key).append("&");
			}
			return s.toString().getBytes("UTF8");
		}
		if(str.equals("/position.txt")){
			return InterfaceHandler.getInstance().getPosition().getBytes();
		}
		if(str.equals("/opmodes.txt")){
			String[] list = InterfaceHandler.getInstance().getOpmodes().toArray(new String[0]);
			OpmodePacket packet = new OpmodePacket();
			packet.opmodes = list;
			return SimpleGson.getInstance().toJson(packet).getBytes();
		}
		if(str.equals("/indexfiles.txt")){
			ArrayList<String> files = VirtualSD.getInstance().indexFiles();
			ArrayList<String> folders = VirtualSD.getInstance().indexFolders();
			FileIndexPacket packet = new FileIndexPacket();
			packet.files = files.toArray(new String[0]);
			packet.folders = folders.toArray(new String[0]);
			return SimpleGson.getInstance().toJson(packet).getBytes();
		}
		if(str.endsWith(".vfile")){
			String s = str.replace(".vfile", "");
			File file = VirtualSD.getInstance().getFile(s).getFile();
			FileInputStream fileInputStreamReader = new FileInputStream(file);
			byte[] bytes = new byte[(int)file.length()];
			fileInputStreamReader.read(bytes);
			if(getContentType(s).equals("image/png")) {
				return new String(Base64.encode(bytes, 0), StandardCharsets.UTF_8).getBytes();
			}else{
				return new String(bytes).getBytes(StandardCharsets.UTF_8);
			}
		}
		InputStream inputStream = InterfaceHandler.getInstance().getInputStream(str);
		ArrayList<Byte> byteArr = new ArrayList<>();
		int read = inputStream.read();
		while(read > -1){
			byteArr.add((byte)read);
			read = inputStream.read();
		}
		byte[] toReturn = new byte[byteArr.size()];
		for(int i = 0; i < byteArr.size(); i ++){
			toReturn[i] = byteArr.get(i);
		}
		return toReturn;
	}

	// return supported MIME Types
	private String getContentType(String fileRequested) {
		if (fileRequested.endsWith(".htm")  ||  fileRequested.endsWith(".html") || !fileRequested.contains(".")) {
			return "text/html";
		} else if (fileRequested.endsWith(".css")) {
			return "text/css";
		}else if(fileRequested.endsWith(".png") || fileRequested.endsWith(".jpg") || fileRequested.endsWith(".jpeg")){
			return "image/png";
		}
		return "text/plain";
	}

	private void fileNotFound(PrintWriter out, OutputStream dataOut, String fileRequested) throws IOException {
		File file = new File(WEB_ROOT, FILE_NOT_FOUND);
		int fileLength = (int) file.length();
		String content = "text/html";
		byte[] fileData = readFileData(FILE_NOT_FOUND);

		out.println("HTTP/1.1 404 File Not Found");
		out.println("Server: Java HTTP Server from SSaurel : 1.0");
		out.println("Date: " + new Date());
		out.println("Content-type: " + content);
		out.println("Content-length: " + fileLength);
		out.println(); // blank line between headers and content, very important !
		out.flush(); // flush character output stream buffer


		dataOut.write(fileData, 0, fileLength);
		dataOut.flush();

		if (verbose) {
			Log.i("Dashboard", "File " + fileRequested + " not found");
		}
	}

	private static String getIPAddress() {
		try {
			List<NetworkInterface> interfaces = Collections.list(NetworkInterface.getNetworkInterfaces());
			for (NetworkInterface intf : interfaces) {
				List<InetAddress> addrs = Collections.list(intf.getInetAddresses());
				for (InetAddress addr : addrs) {
					if (!addr.isLoopbackAddress()) {
						String sAddr = addr.getHostAddress();
						//boolean isIPv4 = InetAddressUtils.isIPv4Address(sAddr);
						boolean isIPv4 = sAddr.indexOf(':')<0;
						if(isIPv4)
							return sAddr;
					}
				}
			}
		} catch (Exception ex) { } // for now eat exceptions
		return "";
	}

}
