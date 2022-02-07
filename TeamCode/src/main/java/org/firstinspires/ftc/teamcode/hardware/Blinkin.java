package org.firstinspires.ftc.teamcode.hardware;


import android.util.Log;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import org.firstinspires.ftc.teamcode.util.Time;

import java.util.concurrent.atomic.AtomicReference;

import static org.firstinspires.ftc.teamcode.util.Init.init;
import static org.firstinspires.ftc.teamcode.util.Time.await;
import static org.firstinspires.ftc.teamcode.util.Time.timeout;

public class Blinkin {

    public static RevBlinkinLedDriver Driver = null;

    public static void setColor(String cname) {

        if (Driver == null) throw new NullPointerException("Blinkin.Driver is Not Defined");

        switch (cname) {
            case "dark gray":  Driver.setPattern(BlinkinPattern.DARK_GRAY); break;
            case "gray":       Driver.setPattern(BlinkinPattern.GRAY); break;
            case "white":      Driver.setPattern(BlinkinPattern.WHITE); break;
            case "violet":     Driver.setPattern(BlinkinPattern.VIOLET); break;
            case "blue":       Driver.setPattern(BlinkinPattern.BLUE); break;
            case "aqua":       Driver.setPattern(BlinkinPattern.AQUA); break;
            case "green":      Driver.setPattern(BlinkinPattern.GREEN); break;
            case "lime":       Driver.setPattern(BlinkinPattern.LIME); break;
            case "yellow":     Driver.setPattern(BlinkinPattern.YELLOW); break;
            case "orange":     Driver.setPattern(BlinkinPattern.ORANGE); break;
            case "red":        Driver.setPattern(BlinkinPattern.RED); break;
            case "hot pink":   Driver.setPattern(BlinkinPattern.HOT_PINK); break;
            default:           Driver.setPattern(BlinkinPattern.BLACK); break;
        }

    }

    /**
     * The 'auto' boolean is optional, just set true for auto
     * @param currentTime time in lmao
     */
    public static synchronized void updateLightTimer(int currentTime) throws InterruptedException {
        updateLightTimer(currentTime, false);
    }
    public static synchronized void updateLightTimer(int currentTime, boolean auto) throws InterruptedException {
        int matchLength = auto ? 30 : 150;
        currentTime = matchLength-(currentTime/1000);
        Log.i("Blinkin.updateLightTimer()", "Count Down : "+currentTime);
        switch(currentTime) {
            case(35):
            case(32):
                Driver.setPattern(BlinkinPattern.RED);
                break;
            case(34):
            case(31):
                Driver.setPattern(BlinkinPattern.BLUE);
                break;
            case(33):
            case(30):
                Driver.setPattern(BlinkinPattern.GREEN);
                break;
            case(5):
            case(3):
            case(1):
                Flash(BlinkinPattern.BLUE, BlinkinPattern.RED, 100);
                break;
            case(4):
            case(2):
            case(0):
                Flash(BlinkinPattern.HOT_PINK, BlinkinPattern.GREEN, 100);
                break;
            default:
                Driver.setPattern(BlinkinPattern.BLACK);
                break;
        }
    }

    /**
     * Flashes between two patterns
     * @param P1 Blinkin Pattern 1
     * @param P2 Blinkin Pattern 2
     * @param duration Time of each flash in ms
     */
    private static void Flash(BlinkinPattern P1, BlinkinPattern P2, int duration) throws InterruptedException {
        //AtomicReference<Integer> newD = new AtomicReference<>(duration);
        init(() -> {
            //newD.set((1000 % (duration * 2)) / 2);
            Log.i("Blinkin.Flash()", String.valueOf(duration));
        });

        for (int i = 0; (1000 / (duration * 2)) > i; i++) {
            await(duration, () -> {
                Driver.setPattern(P1);
                Log.i("Blinkin.Flash()", "Flashed P1");
            });
            Time.reset();
            await(duration, () -> {
                Driver.setPattern(P2);
                Log.i("Blinkin.Flash()", "Flashed P2");
            });
            Time.reset();
        }
    }

}