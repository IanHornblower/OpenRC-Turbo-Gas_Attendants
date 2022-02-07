package org.firstinspires.ftc.teamcode.opmodes.Testing.RevHub;

import android.util.Log;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.hardware.Battery;
import org.firstinspires.ftc.teamcode.hardware.Blinkin;
import org.firstinspires.ftc.teamcode.hardware.Controller;
import org.openftc.revextensions2.ExpansionHubEx;
import org.firstinspires.ftc.teamcode.util.Color;

import static com.qualcomm.robotcore.hardware.Gamepad.ID_UNASSOCIATED;

@Disabled
@Config
@TeleOp(name="Controller Test", group="Testing")
public class ControllerTest extends LinearOpMode {

    public static boolean DashTelemetryEnabled = true;
    public double time = 0;
    public double cycleLength = 0;
    public static double timeRunning = 0;
    public static String BuildNumber = "2.5.3";
    public static String BuildComment = "Updated Blinkin Timer for the 24th time";
    public static String HubName = "Control Hub";
    public static boolean ImperialUnits = false;
    public static String timeUnit = "HOURS";
    public static boolean pain = false;
    ExpansionHubEx expansionHub;
    RevBlinkinLedDriver Driver;


    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        if(DashTelemetryEnabled) telemetry = dashboard.getTelemetry();
        telemetry.clear();
        expansionHub = hardwareMap.get(ExpansionHubEx.class, HubName);
        Driver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        Blinkin.Driver = Driver;
        Battery.expansionHub1 = expansionHub;
        Log.i("GP-A", Float.toString(gamepad1.left_stick_x));
        Gamepad gp1 = gamepad1;
        Gamepad gp2 = gamepad2;
        Controller gamepad1 = new Controller(gp1);
        Controller gamepad2 = new Controller(gp2);
        telemetry.addLine("Controller Test");
        telemetry.addData("Version", BuildNumber);
        telemetry.addData("Ver Com", BuildComment);
        if (gamepad1.type() != Gamepad.Type.SONY_PS4 && gamepad1.id != ID_UNASSOCIATED) telemetry.addLine("Controller 1 isn't a PS4 Controller!");
        if (gamepad2.type() != Gamepad.Type.SONY_PS4 && gamepad2.id != ID_UNASSOCIATED) telemetry.addLine("Controller 2 isn't a PS4 Controller!");
        if (gamepad1.id == ID_UNASSOCIATED) telemetry.addLine("Controller 1 isn't Connected!");
        if (gamepad2.id == ID_UNASSOCIATED) telemetry.addLine("Controller 2 isn't Connected!");
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

            if(pain) throw new OutOfMemoryError("\uD80C\uDD89");

            telemetry.clear();


            // Gamepad Testing Here
            gamepad1.update();

            if(gamepad1.touchingTopLeft) telemetry.addData("Touch Region", "Top Left");
            if(gamepad1.touchingTopRight) telemetry.addData("Touch Region", "Top Right");
            if(gamepad1.touchingBottomLeft) telemetry.addData("Touch Region", "Bottom Left");
            if(gamepad1.touchingBottomRight) telemetry.addData("Touch Region", "Bottom Right");

            telemetry.addData("Gamepad Loc 1", gamepad1.loc1[0]+", "+gamepad1.loc1[1]);
            telemetry.addData("Gamepad Loc 2", gamepad1.loc2[0]+", "+gamepad1.loc2[1]);
            telemetry.addData("Gamepad Loc 3", gamepad1.loc3[0]+", "+gamepad1.loc3[1]);



            if (gamepad1.right_trigger > 0)
                gamepad1.rumble((int)((2*cycleLength)/10*(gamepad1.right_trigger*10)));



            // Control Hub LED Testing
            if (gamepad1.circle) { expansionHub.setLedColor(255,0,0); Blinkin.setColor("red"); }
            else if (gamepad1.square) { expansionHub.setLedColor(255,0,255); Blinkin.setColor("hot pink"); }
            else if (gamepad1.cross) { expansionHub.setLedColor(0,0,255); Blinkin.setColor("blue"); }
            else if (gamepad1.triangle) { expansionHub.setLedColor(0,255,0);  Blinkin.setColor("green"); }
            else if (gamepad1.touchpad_finger_1) {
                double p1 = gamepad1.touchpad_finger_1_x;
                int p2 = (int)Math.round(((((100*p1)/2)+50)/100)*360);
                telemetry.addData("F1.X to Hue", p2);
                int[] pf = Color.HSVtoRGB((float)p2/360,1,1);
                telemetry.addData("R", pf[0]);
                telemetry.addData("G", pf[1]);
                telemetry.addData("B", pf[2]);
                expansionHub.setLedColor(pf[0], pf[1], pf[2]);
            } else expansionHub.setLedColor(255,255,255);




            telemetry.addData("1 Finger", gamepad1.touchpad_finger_1);
            telemetry.addData("2 Finger", gamepad1.touchpad_finger_2);
            telemetry.addData("1 Finger X", gamepad1.touchpad_finger_1_x);
            telemetry.addData("1 Finger Y", gamepad1.touchpad_finger_1_y);
            telemetry.addData("2 Finger X", gamepad1.touchpad_finger_2_x);
            telemetry.addData("2 Finger Y", gamepad1.touchpad_finger_2_y);
            telemetry.addData("Gamepad ID", gamepad1.getUser());
            telemetry.addData("Loop Time", cycleLength+"ms");
            telemetry.addData("Time Running", (int)Math.floor((double)(timeRunning/1000))+" Seconds");
            telemetry.addData("Battery Voltage", Battery.voltage());
            telemetry.addData("Battery Current", Math.round(Battery.currentDraw())+"mA");
            telemetry.addData("Battery Apx. %", Battery.percentage()+"%");
            telemetry.addData("Battery Time", Battery.timeRemaining(timeUnit));
            temp = ImperialUnits ? expansionHub.getInternalTemperature(ExpansionHubEx.TemperatureUnits.FAHRENHEIT) + "°F" : expansionHub.getInternalTemperature(ExpansionHubEx.TemperatureUnits.CELSIUS) + "°C";
            telemetry.addData("Temperature", temp);

            Blinkin.updateLightTimer((int)timeRunning);

            telemetry.update();



        }
    }

}
