package MathSystems;

public class PIDSystem {
    private double kp, ki, kd, target, error, previousError;
    private double proportional, integral, derivative;
    private float dt;

    public PIDSystem(double kp, double ki, double kd){
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        target = 0;
        error = 0;
        previousError = 0;
        proportional = 0;
        integral = 0;
        derivative = 0;
        dt = 0;
    }

    public PIDSystem(double kp, double ki, double kd, double target){
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.target = target;
        error = 0;
        previousError = 0;
        proportional = 0;
        integral = 0;
        derivative = 0;
        dt = 0;
    }

    public double getCorrection(double current){
        float start = System.currentTimeMillis();
        error = target - current;
        proportional = error * kp;
        if(dt != 0){
            integral += error * ki * dt;
            if(previousError != 0){
                derivative = kd * (error - previousError) / dt;
            }
        }

        dt = System.currentTimeMillis() - start;
        previousError = error;

        return proportional + integral + derivative;
    }

    public void setTarget(double target) {
        this.target = target;
    }

    public double getTarget() {
        return target;
    }

    public void reset(){
        error = 0;
        previousError = 0;
        proportional = 0;
        integral = 0;
        derivative = 0;
        dt = 0;
    }
}
