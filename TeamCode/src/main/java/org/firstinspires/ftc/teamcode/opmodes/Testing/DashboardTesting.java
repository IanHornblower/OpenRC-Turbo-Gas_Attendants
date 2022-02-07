package org.firstinspires.ftc.teamcode.opmodes.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.dashboard.Field;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.vision.FreightFrenzyCamera;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;

@Disabled
@Autonomous(name = "DashboardTest", group = "Testing")
public class DashboardTesting extends LinearOpMode {

    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        //FreightFrenzyCamera camera = new FreightFrenzyCamera(hardwareMap);

        double x = 0, y = 0, h = Math.toRadians(90);

        telemetry.addLine("Waiting for Init");
        telemetry.update();

        telemetry.clear();
        telemetry.addLine("Camera Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            //camera.refreshScalars();

            TelemetryPacket packet = new TelemetryPacket();

            x -= gamepad1.left_stick_y/5000;
            y -= gamepad1.left_stick_x/5000;

            h -= Math.toRadians(gamepad1.right_stick_x/5000);

            //telemetry.addData("Location", camera.sDeterminePosition());
            //telemetry.update();

            Field field = new Field(packet);

            field.createRobot(new Pose2D(x, y, h));


            dashboard.sendTelemetryPacket(field.getPacket());
        }
    }
}