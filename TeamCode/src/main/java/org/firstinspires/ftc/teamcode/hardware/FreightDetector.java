package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class FreightDetector {

    public static double threshold = 60;
    public static volatile boolean isRunning = true;

    Robot robot;
    ColorSensor freightDetector_color;
    DistanceSensor freightDetector_distance;

    public FreightDetector(Robot robot) {
        this.robot = robot;
        freightDetector_color = robot.getFreightSensor_color();
        freightDetector_distance = robot.getFreightSensor_distance();
    }

    public double distance() {
        return robot.getFreightSensor_distance().getDistance(DistanceUnit.MM);
    }

    public boolean hasFreight() {
        return distance() < threshold;
    }

    public void run() {
        new Thread(()-> {
            boolean isGreen = false;

            while(isRunning) {
                isGreen = hasFreight();

                if(hasFreight()  ) {
                    robot.blinkin.setColor("green");
                }
                else {
                    robot.blinkin.setColor("red");
                }
            }
            while(!isRunning) {
                robot.blinkin.setColor("green");
            }
        }).start();
    }

    public void close() {
        isRunning = false;
    }
}
