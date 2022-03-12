package org.firstinspires.ftc.teamcode.hardware;

import android.util.Log;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;

import java.util.concurrent.atomic.AtomicReference;

import static org.firstinspires.ftc.teamcode.util.Init.init;

public class Blinkin {

    Robot robot;
    public RevBlinkinLedDriver Driver;

    public Blinkin(Robot robot) {
        this.robot = robot;
        Driver = robot.getBlinkin();
    }

    public void setColor(String cname) {

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
    public synchronized void updateLightTimer(int currentTime) throws InterruptedException {
        updateLightTimer(currentTime, false);
    }
    public synchronized void updateLightTimer(int currentTime, boolean auto) throws InterruptedException {
        int matchLength = auto ? 30 : 150;
        currentTime = matchLength-(currentTime/1000);
        Log.i("Blinkin.updateLightTimer()", "Count Down : "+currentTime);
        switch(currentTime) {
            case(35):
            case(32):
                //setPattern(BlinkinPattern.RED);
                break;
            case(34):
            case(31):
                //setPattern(BlinkinPattern.BLUE);
                break;
            case(33):
            case(30):
                //setPattern(BlinkinPattern.GREEN);
                break;
            case(5):
            case(3):
            case(1):
                break;
            case(4):
            case(2):
            case(0):
                break;
            default:
                //setPattern(BlinkinPattern.BLACK);
                break;
        }
    }

}
