package org.firstinspires.ftc.robotcontroller.internal.WebInterface.VariableConfig;

import com.google.gson.annotations.SerializedName;

public enum VariableType {
    @SerializedName("boolean")
    BOOLEAN,

    @SerializedName("int")
    INT,

    @SerializedName("double")
    DOUBLE,

    @SerializedName("string")
    STRING,

    @SerializedName("enum")
    ENUM,

    @SerializedName("custom")
    CUSTOM;

    public static VariableType fromClass(Class<?> c) {
        if (c == Boolean.class || c == boolean.class) {
            return BOOLEAN;
        } else if (c == Integer.class || c == int.class) {
            return INT;
        } else if (c == Double.class || c == double.class) {
            return DOUBLE;
        } else if (c == String.class) {
            return STRING;
        } else if (c.isEnum()) {
            return ENUM;
        } else {
            return CUSTOM;
        }
    }
}
