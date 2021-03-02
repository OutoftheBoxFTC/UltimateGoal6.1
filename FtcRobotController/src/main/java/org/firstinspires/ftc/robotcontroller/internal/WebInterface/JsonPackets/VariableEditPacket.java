package org.firstinspires.ftc.robotcontroller.internal.WebInterface.JsonPackets;

public class VariableEditPacket {

    public DoubleValuePacket[] numvars;
    public BooleanValuePacket[] boolvars;
    public StringValuePacket[] strvars;

    public class DoubleValuePacket{
        public String name;
        public Double val;
    }

    public class BooleanValuePacket{
        public String name;
        public boolean val;
    }

    public class StringValuePacket{
        public String name;
        public String val;
    }
}
