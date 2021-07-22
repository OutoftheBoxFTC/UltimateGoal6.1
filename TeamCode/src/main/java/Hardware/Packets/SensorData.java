package Hardware.Packets;

/**
 * Sensor Data Packet
 * Contains variables retrieved from sensors. Defaults all variables to 0
 */

public class SensorData {
    private double gyro, gyro2, fps, wobbleLift, backlog, rings, heading, pitch, range, distance;
    private double powershots[] = new double[]{0, 0, 0};
    private int odometryLeft, odometryRight, odometryAux;
    private boolean track = false;
    private long timestamp;
    public SensorData(){
        odometryAux = 0;
        odometryLeft = 0;
        odometryRight = 0;
        gyro = 0;
        gyro2 = 0;
        wobbleLift = 0;
        backlog = 0;
        rings = 0;
        heading = 0;
        pitch = 0;
        range = 0;
        distance = 9999;
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

    public void setWobbleLift(double wobbleLift) {
        this.wobbleLift = wobbleLift;
    }

    public double getWobbleLift() {
        return wobbleLift;
    }

    public double getBacklog() {
        return backlog;
    }

    public void setBacklog(double backlog) {
        this.backlog = backlog;
    }

    public void setRings(double rings) {
        this.rings = rings;
    }

    public double getRings() {
        return rings;
    }

    public double getFps() {
        return fps;
    }

    public double getHeading() {
        return heading;
    }

    public double getPitch() {
        return pitch;
    }

    public double getRange() {
        return range;
    }

    public boolean getTrack(){
        return track;
    }

    public double[] getPowershots() {
        return powershots;
    }

    public void setHeading(double heading) {
        this.heading = heading;
    }

    public void setPitch(double pitch) {
        this.pitch = pitch;
    }

    public void setRange(double range) {
        this.range = range;
    }

    public void setTrack(boolean track) {
        this.track = track;
    }

    public void setPowershots(double[] powershots) {
        this.powershots = powershots;
    }

    public void setDistance(double distance) {
        this.distance = distance;
    }

    public double getDistance() {
        return distance;
    }

    public void setFps(double fps) {
        this.fps = fps;
    }

    public void setTimestamp(long timestamp) {
        this.timestamp = timestamp;
    }

    public long getTimestamp() {
        return timestamp;
    }
}
