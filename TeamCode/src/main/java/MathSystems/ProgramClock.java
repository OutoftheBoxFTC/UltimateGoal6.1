package MathSystems;
public class ProgramClock {
    private static ProgramClock instance = new ProgramClock();
    public long dt, prevTime;

    public ProgramClock(){
        dt = 0;
        prevTime = 0;
    }

    public void internalUpdate(){
        long start = System.nanoTime();
        if(prevTime == 0){
            prevTime = start;
        }
        dt = start - prevTime;
        prevTime = start;
    }

    public static void update(){
        instance.internalUpdate();
    }

    public static long getFrameTimeNano(){
        return instance.dt;
    }

    public static double getFrameTimeMillis(){
        return MathUtils.nanoToMillis(instance.dt);
    }

    public static double getFrameTimeSeconds(){
        return MathUtils.nanoToDSec(instance.dt);
    }
}