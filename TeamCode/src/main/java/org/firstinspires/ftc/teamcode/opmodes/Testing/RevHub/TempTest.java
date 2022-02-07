    package org.firstinspires.ftc.teamcode.opmodes.Testing.RevHub;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.Battery;
import org.openftc.revextensions2.ExpansionHubEx;

import java.io.File;
import java.io.InputStream;
import java.util.zip.DeflaterOutputStream;
import java.io.FileOutputStream;



import java.math.BigInteger;

@Disabled
@TeleOp(name="Temperature Test", group="Testing")
@Config
public class TempTest extends LinearOpMode {

    public static boolean DashTelemetryEnabled = true;
    public long time = 0;
    public long cycleLength = 0;
    public long timeRunning = 0;
    public static int BuildNumber = 23;
    public static String HubName = "Expansion Hub 1";
    public static boolean ImperialUnits = false;
    public static boolean StressTest = false;
    public static int StressType = 0;
    protected long stressNum = 0;
    int i = 0;
    ExpansionHubEx expansionHub;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.clear();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        if(DashTelemetryEnabled) telemetry = dashboard.getTelemetry();
        expansionHub = hardwareMap.get(ExpansionHubEx.class, HubName);
        Battery.expansionHub1 = expansionHub;
        expansionHub.setLedColor(0,0,0);

        telemetry.addLine("Controller Test");
        telemetry.addData("Version", BuildNumber);
        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        telemetry.clear();
        time = System.currentTimeMillis();
        cycleLength = 0;
        timeRunning = 0;
        String temp = "";

        while(opModeIsActive()) {

            cycleLength = System.currentTimeMillis() - time;

            time = System.currentTimeMillis();
            timeRunning = timeRunning + cycleLength;

            telemetry.clear();

            telemetry.addData("Loop Time", cycleLength+"ms");
            telemetry.addData("Time Running", (int)Math.floor((double)(timeRunning/1000))+" Seconds");
            telemetry.addData("Battery Voltage", Battery.voltage());
            telemetry.addData("Battery Current", Math.round(Battery.currentDraw())+"mA");
            telemetry.addData("Battery Apx. %", Battery.percentage()+"%");
            if (StressTest) telemetry.addData("Stress Test Number", stressNum);
            if (StressTest) telemetry.addData("Stress Test Iteration", i);
            if(ImperialUnits)
                temp = expansionHub.getInternalTemperature(ExpansionHubEx.TemperatureUnits.FAHRENHEIT) + "°F";
            else temp = expansionHub.getInternalTemperature(ExpansionHubEx.TemperatureUnits.CELSIUS) + "°C";
            telemetry.addData("Temperature", temp);
            if (gamepad1.a) telemetry.addLine("Controller Works lmao");
            telemetry.update();

            if (StressTest) {
                switch (StressType) {
                    case 0:
                        if (i > 1000) i = 1;
                        stressNum = (long)((float)Math.pow(7, i)*(float)7);
                        i++;
                        break;
                    case 1:
                        if (i > 25 ) i = 1;
                        stressNum = Long.parseLong(String.valueOf(fib(new BigInteger(String.valueOf(i)))));
                        i++;
                        break;
                    case 2:
                        if (i > 100000) i = 1;
                        if(i%2 == 0) // if the remainder of `i/2` is 0
                            stressNum += -1 / ( 2 * i - 1);
                        else
                            stressNum += 1 / (2 * i - 1);
                        i++;
                        break;
                    default:
                        if (i > 1000) i = 1;
                        stressNum = i^2;
                        i++;
                        break;
                }

            }



        }
    }

    public static BigInteger fib(BigInteger n) {
        if (n.compareTo(BigInteger.ONE) == -1 || n.compareTo(BigInteger.ONE) == 0 ) return n;
        else
            return fib(n.subtract(BigInteger.ONE)).add(fib(n.subtract(BigInteger.ONE).subtract(BigInteger.ONE)));
    }

}
