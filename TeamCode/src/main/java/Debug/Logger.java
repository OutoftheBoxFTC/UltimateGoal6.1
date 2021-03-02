package Debug;

import org.firstinspires.ftc.robotcontroller.internal.WebInterface.VariableConfig.Configurable;

import java.io.BufferedOutputStream;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.ArrayList;
import java.util.HashMap;

public class Logger{

    public static Logger instance = new Logger();

    HashMap<String, Double> vals;
    ServerSocket socket;
    ArrayList<Socket> sockets;

    public Logger() {
        vals = new HashMap<>();
        try {
            socket = new ServerSocket(9999);
        } catch (IOException e) {
            e.printStackTrace();
        }
        this.sockets = new ArrayList<>();
        Thread t = new Thread(){
            @Override
            public void run() {
                try {
                    sockets.add(socket.accept());
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        };
        t.start();
    }

    public void add(String name, Double val){
        this.vals.put(name, val);
    }

    public void update(){
        for(Socket s : sockets){
            try {
                BufferedOutputStream outputStream = new BufferedOutputStream(s.getOutputStream());
                StringBuilder message = new StringBuilder();
                for(String str : vals.keySet()){
                    message.append(str).append(",").append(vals.get(str)).append("/");
                }
                outputStream.write(message.toString().getBytes());
                outputStream.flush();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    public static Logger getInstance() {
        return instance;
    }
}
