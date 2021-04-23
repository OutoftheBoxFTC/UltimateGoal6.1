package org.firstinspires.ftc.robotcontroller.internal.WebInterface.VSD;

import android.content.Context;

import com.qualcomm.robotcore.util.RobotLog;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

public class VFile {
    private String fileName;
    private static final String PATH = System.getenv("EXTERNAL_STORAGE") + "/FIRST/Dashboard/";
    private File file;
    private BufferedReader inputStream;
    private BufferedWriter outputStream;
    private Context context;
    public VFile(String fileName, Context context) throws IOException {
        this.fileName = fileName;
        this.context = context;
        file = new File(PATH + this.fileName);
        file.getParentFile().mkdirs();
        file.createNewFile();
        RobotLog.ii("BruhPath", file.getAbsolutePath());
        inputStream = new BufferedReader(new FileReader(file));
        outputStream = new BufferedWriter(new FileWriter(file, true));
    }

    public String read() throws IOException {
        String total = "", tmp = "";
        tmp = inputStream.readLine();
        while(tmp != null){
            total += tmp + '\n';
            tmp = inputStream.readLine();
        }
        inputStream = new BufferedReader(new FileReader(file));
        return total;
    }

    public void write(String s) throws IOException {
        outputStream.append(s);
        outputStream.flush();
    }

    public void writeLine(String s) throws IOException {
        outputStream.append(s + '\n');
        outputStream.flush();
    }

    public void clear() throws IOException {
        new PrintWriter(file).close();
    }

    public void deleteFile() throws IOException {
        file.delete();
    }

    public String getFileName(){
        return fileName;
    }

    public File getFile() {
        return file;
    }
}
