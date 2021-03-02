package org.firstinspires.ftc.robotcontroller.internal.WebInterface.VariableConfig;

import android.content.Context;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.IOException;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

import dalvik.system.DexFile;

public class ClassHandler {
    private static final String TAG = "ClassHandler";

    private DexFile dex;

    private HashMap<String, UpdateableVariable> variables;

    public ClassHandler() throws IOException {
        variables = new HashMap<>();

        Context context = AppUtil.getInstance().getApplication();
        dex = new DexFile(context.getPackageCodePath());

        List<String> classes = Collections.list(dex.entries());

        ClassLoader loader = ClassHandler.class.getClassLoader();

        RobotLog.ii(TAG, "Started");

        for(String name : classes){
            try {
                Class cl = Class.forName(name, false, loader);
                if(cl.isAnnotationPresent(Configurable.class)){
                    RobotLog.ii(TAG, "Found class: " + cl.getSimpleName());
                    Field[] fieldArr = cl.getFields();
                    for(Field f : fieldArr){
                        addVars(f, cl.getSimpleName());
                    }
                }
            } catch (ClassNotFoundException e) {

            }
        }

        for(String s : variables.keySet()){
            RobotLog.ii(TAG, s);
        }
    }

    private void addVars(Field f, String name){
        if(!Modifier.isStatic(f.getModifiers()) || Modifier.isFinal(f.getModifiers())){
            return;
        }
        Class fClass = f.getType();
        VariableType type = VariableType.fromClass(fClass);
        if(type.equals(VariableType.CUSTOM)){
            for(Field subField : fClass.getFields()){
                if(Modifier.isFinal(f.getModifiers())){
                    continue;
                }
                addVars(subField, name + "." + fClass.getCanonicalName());
            }
        }else{
            variables.put(name + "." + f.getName(), new UpdateableVariable(type, f, null));
        }
    }

    public HashMap<String, UpdateableVariable> getVariables(){
        return variables;
    }

    public void set(String variable, Object value){
        variables.get(variable).set(value);
    }
}
