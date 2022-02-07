package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.control.Executor;

import java.util.concurrent.atomic.AtomicBoolean;

public class Init {
    private final AtomicBoolean done = new AtomicBoolean();
    static boolean init = false;

    public Init() {
    }

    public static void reset() {
        init = false;
    }

    public void run(Executor x) throws InterruptedException {
        if (done.get()) return;
        if (done.compareAndSet(false, true)) {
            x.method();
        }
    }

    /**
     * run x only once.
     * @param x single or set of functions only run once even in a loop
     */
    public static void init(Executor x) throws InterruptedException {
        if(!init) {
            x.method();
            init = true;
        }
    }

}