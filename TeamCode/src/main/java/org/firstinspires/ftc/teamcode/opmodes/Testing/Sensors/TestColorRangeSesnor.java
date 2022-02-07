package org.firstinspires.ftc.teamcode.opmodes.Testing.Sensors;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.LowPassFilter;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.control.CornettCore;
import org.firstinspires.ftc.teamcode.control.Trajectory;
import org.firstinspires.ftc.teamcode.hardware.Blinkin;
import org.firstinspires.ftc.teamcode.hardware.FreightDetector;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.AngleUtil;

@Disabled
@Config
@TeleOp(name = "Color Testing", group = "Testing")
public class TestColorRangeSesnor extends LinearOpMode {

    public static double gain = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        CornettCore motionProfile = new CornettCore(robot);

        Blinkin.Driver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        telemetry = robot.dashboard.getTelemetry();

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {

            if(gamepad1.cross) {
                Blinkin.setColor("blue");
            }
            if(gamepad1.circle) {
                Blinkin.setColor("red");
            }
            if(gamepad1.triangle) {
                Blinkin.setColor("green");
            }
            if(gamepad1.square) {
                Blinkin.setColor("hot pink");
            }

            double distance = robot.getFreightSensor_distance().getDistance(DistanceUnit.MM);
            LowPassFilter lowPassDistance = new LowPassFilter(gain);

            robot.freightDetector.run();

            telemetry.addData("Raw Distance", distance);
            telemetry.addData("Low Pass Distance", lowPassDistance.estimate(distance));
            telemetry.update();

        }

    }
}
