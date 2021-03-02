package org.firstinspires.ftc.robotcontroller.internal.WebInterface.JsonPackets;

import java.util.HashMap;

public class VariablesPacket {
    int opcode = 3;
    public String[] numVarNames;
    public Double[] numVars;
    public String[] boolVarsNames;
    public Boolean[] boolVars;
    public String[] strVarNames;
    public String[] strVars;
}
