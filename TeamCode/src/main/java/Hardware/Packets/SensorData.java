package Hardware.Packets;

/**
 * Sensor Data Packet
 * Contains variables retrieved from sensors. Defaults all variables to 0
 */

public class SensorData {
    private double gyro, gyro2, fps;
    private int odometryLeft, odometryRight, odometryAux;
    public SensorData(){
        odometryAux = 0;
        odometryLeft = 0;
        odometryRight = 0;
        gyro = 0;
        gyro2 = 0;
    }

    public int getOdometryAux() {
        return odometryAux;
    }

    public void setOdometryAux(int odometryAux) {
        this.odometryAux = odometryAux;
    }

    public int getOdometryLeft() {
        return odometryLeft;
    }

    public void setOdometryLeft(int odometryLeft) {
        this.odometryLeft = odometryLeft;
    }

    public void setOdometryRight(int odometryRight) {
        this.odometryRight = odometryRight;
    }

    public int getOdometryRight() {
        return odometryRight;
    }

    public double getGyro() {
        return gyro;
    }

    public void setGyro(double gyro) {
        this.gyro = gyro;
    }

    public double getGyro2() {
        return gyro2;
    }

    public void setGyro2(double gyro2) {
        this.gyro2 = gyro2;
    }

    public double getFps() {
        return fps;
    }

    public void setFps(double fps) {
        this.fps = fps;
    }
}
