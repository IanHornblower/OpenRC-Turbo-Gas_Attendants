package org.firstinspires.ftc.teamcode.util;

public class Timer {

    private long startTime;

    public Timer() {
        startTime = 0;
    }

    public void addSeconds(double seconds) {
        startTime = startTime-(long)(seconds*1e+9);
    }
    
    public void removeSeconds(double seconds) {
        startTime = startTime+(long)(seconds*1e+9);
    }

    public void start() {
        startTime = (long) (System.nanoTime());
    }

    public void reset() {
        startTime = (long) (System.nanoTime());
    }

    public double currentMills() {
        return (System.nanoTime() - startTime)/1e-6;
    }

    public double currentSeconds() {
        return (System.nanoTime() - startTime)*1e-9;
    }
}
