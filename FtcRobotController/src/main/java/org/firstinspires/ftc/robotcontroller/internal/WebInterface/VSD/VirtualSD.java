package org.firstinspires.ftc.robotcontroller.internal.WebInterface.VSD;

import android.content.Context;

import com.qualcomm.robotcore.util.RobotLog;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

public class VirtualSD {
    private static VirtualSD instance = new VirtualSD();
    private static final String PATH = System.getenv("EXTERNAL_STORAGE") + "/FIRST/Dashboard/";
    private Context context;
    private ArrayList<VFile> files;
    private VirtualSD(){
        files = new ArrayList<>();
    }

    public void addContext(Context context){
        this.context = context;
        String s = "";
    }

    public VFile getFile(String fileName) throws IOException {
        for(VFile file : files){
            if(file.getFileName().equals(fileName)){
                return file;
            }
        }
        VFile file = new VFile(fileName, context);
        files.add(file);
        return file;
    }

    public void addFolder(String folderName) throws IOException {
        VFile file = new VFile(folderName + "/" + "creation", context);
        file.deleteFile();
    }

    public void deleteFolder(String folderName){
        new File(PATH + folderName).delete();
    }

    public static VirtualSD getInstance(){
        return instance;
    }

    public ArrayList<String> indexFiles(){
        ArrayList<String> files = new ArrayList<>();
        ArrayList<String> rawFiles = new ArrayList<>();
        listFiles(".", rawFiles);
        if(rawFiles != null && rawFiles.size() > 0) {
            for (String f : rawFiles) {
                files.add(f.replace(PATH, "").replace("./", ""));
            }
        }
        return files;
    }

    public ArrayList<String> indexFolders(){
        ArrayList<String> files = new ArrayList<>();
        ArrayList<String> rawFiles = new ArrayList<>();
        listFolders(".", rawFiles);
        if(rawFiles != null && rawFiles.size() > 0) {
            for (String f : rawFiles) {
                files.add(f.replace(PATH, "").replace("./", ""));
            }
        }
        return files;
    }

    public void listFiles(String directoryName, ArrayList<String> files) {
        File directory = new File(PATH + directoryName);

        // Get all files from a directory.
        File[] fList = directory.listFiles();
        if(fList != null) {
            for (File file : fList) {
                //RobotLog.ii("File Found!", file.getName());
                if (file.isFile()) {
                    files.add(file.toString());
                } else if (file.isDirectory()) {
                    RobotLog.ii("File Found!", directory + "/" + file.getName());
                    listFiles(directoryName + "/" + file.getName(), files);
                }
            }
        }
    }

    public void listFolders(String directoryName, ArrayList<String> files) {
        File directory = new File(PATH + directoryName);

        // Get all files from a directory.
        File[] fList = directory.listFiles();
        if(fList != null) {
            for (File file : fList) {
                //RobotLog.ii("Folder Found!", file.getName());
                if (file.isDirectory()) {
                    files.add(file.toString());
                    RobotLog.ii("Folder Found!", directory + "/" + file.getName());
                    listFolders(directoryName + "/" + file.getName(), files);
                }
            }
        }
    }
}
