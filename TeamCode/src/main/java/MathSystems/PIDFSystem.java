package MathSystems;

import com.qualcomm.robotcore.util.RobotLog;

import Debug.Logger;

public class PIDFSystem {
    private double kp, ki, kd, kf, target, error, previousError, dt;
    private double proportional, integral, derivative;
    private long prevTime;

    public PIDFSystem(double kp, double ki, double kd, double kf){
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;
        target = 0;
        error = 0;
        previousError = 0;
        proportional = 0;
        integral = 0;
        derivative = 0;
        dt = 0;
    }

    public PIDFSystem(double kp, double ki, double kd, double kf, double target){
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;
        this.target = target;
        error = 0;
        previousError = 0;
        proportional = 0;
        integral = 0;
        derivative = 0;
        dt = 0;
    }

    public double getCorrection(double error, double feedforward){
        proportional = error * kp;
        if(dt != 0){
            if(error < 10)
                integral += error * ki * dt;
            if(previousError != 0){
                derivative = kd * (Math.abs(error - previousError) / dt);
                if((previousError < 0 && error > 0) || (previousError > 0 && error < 0)){
                    integral = 0;
                }
            }
        }
        if(prevTime == 0)
            prevTime = System.currentTimeMillis();
        dt = MathUtils.millisToSec(System.currentTimeMillis() - prevTime);
        prevTime = System.currentTimeMillis();
        previousError = error;

        RobotLog.ii("PID", proportional + " | " + integral + " | " + derivative + " | " + dt);

        //Logger.getInstance().add("P", proportional);
        //Logger.getInstance().add("I", integral);
        //Logger.getInstance().add("D", derivative);
        //Logger.getInstance().add("Error", Math.toDegrees(error));

        return proportional + integral - derivative + (feedforward * kf);
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
