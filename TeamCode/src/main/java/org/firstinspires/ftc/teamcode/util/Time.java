package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.control.Executor;

import static org.firstinspires.ftc.teamcode.util.Init.init;

public class Time {
    static double startTime = 0;
    static double currentTime = 0;
    static double previousTime = 0;
    static double timeSinceStart = 0;
    static double deltaTime = 0;

    public Time() {
        startTime = 0;
        currentTime = 0;
        previousTime = 0;
        timeSinceStart = 0;
        deltaTime = 0;
    }

    public static void reset() {
        Init.reset();
        startTime = 0;
        currentTime = 0;
        previousTime = 0;
        timeSinceStart = 0;
        deltaTime = 0;
    }

    /**
     * Waits for "mills" milliseconds then runs whatever x contains
     * @param mills amount of time till execution
     * @param x method to be executed at "await" milliseconds
     */

    public static void await(double mills, Executor x) throws InterruptedException {
        init(()-> startTime = System.nanoTime());

        Thread t1 = new Thread(() -> {
            while(timeSinceStart < mills*1e6) {
                currentTime = System.nanoTime();
                timeSinceStart = currentTime - startTime;
            }
            try {
                x.method();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        });

        t1.start();
    }

    /**
     * Waits for "mills" milliseconds then runs whatever x contains
     * @param mills amount of time till execution
     * @param x method to be executed at "await" milliseconds
     */

    public static void wait(double mills, Executor x) throws InterruptedException {
        init(()-> startTime = System.nanoTime());

        try {
            x.method();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        while(timeSinceStart < mills*1e6) {
            currentTime = System.nanoTime();
            timeSinceStart = currentTime - startTime;
        }
    }

    /**
     * Synchronous Function which run for "mills" milliseconds then stop
     * @param mills duration of milliseconds
     * @param x methods to be executed during loop
     */

    public static void timeout(double mills, Executor x) throws InterruptedException {
        init(()-> startTime = System.nanoTime());

        while(timeSinceStart < mills*1e6) {
            currentTime = System.nanoTime();
            timeSinceStart = currentTime - startTime;

            x.method();
        }
    }

    /**
     * Asynchronous Function which run for "mills" milliseconds then stop
     * @param mills duration of milliseconds
     * @param x methods to be executed during loop
     */

    public static void asyncTimeout(double mills, Executor x) throws InterruptedException {
        init(()-> startTime = System.nanoTime());

        Thread t1 = new Thread(() -> {
            while(timeSinceStart < mills*1e6) {
                currentTime = System.nanoTime();
                timeSinceStart = currentTime - startTime;

                try {
                    x.method();
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        });

        t1.start();
    }

    /**
     * Used with a while to
     * @param mills duration of milliseconds
     * @return returns true if time from start is less the "mills"
     */

    public static boolean timeout(double mills) throws InterruptedException {
        init(()-> startTime = System.nanoTime());

        while(timeSinceStart < mills*1e6) {
            currentTime = System.nanoTime();
            timeSinceStart = currentTime - startTime;

            return true;
        }

        return false;
    }


}