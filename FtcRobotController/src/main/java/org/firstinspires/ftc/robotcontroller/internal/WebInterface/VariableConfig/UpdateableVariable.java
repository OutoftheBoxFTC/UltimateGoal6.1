package org.firstinspires.ftc.robotcontroller.internal.WebInterface.VariableConfig;

import java.lang.reflect.Field;

public class UpdateableVariable {
    private VariableType type;
    private Field field;
    private Object parent;

    public UpdateableVariable(VariableType type, Field field, Object parent){
        this.type = type;
        this.field = field;
        this.parent = parent;
    }

    public void set(Object value){
        try {
            field.set(parent, value);
        } catch (IllegalAccessException e) {
            e.printStackTrace();
        }
    }

    public Object get(){
        try {
            return field.get(parent);
        } catch (IllegalAccessException e) {
            e.printStackTrace();
            return null;
        }
    }

    public VariableType getType() {
        return type;
    }
}
