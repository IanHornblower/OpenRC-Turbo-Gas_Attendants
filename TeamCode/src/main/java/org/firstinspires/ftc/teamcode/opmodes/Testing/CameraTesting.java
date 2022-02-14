package org.firstinspires.ftc.teamcode.opmodes.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.teamcode.dashboard.Field;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.vision.FreightFrenzyCamera;

@Autonomous(name = "Camera Test", group = "Testing")
public class CameraTesting extends LinearOpMode {

    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        FreightFrenzyCamera camera = new FreightFrenzyCamera(hardwareMap);

        telemetry.addLine("Waiting for Init");
        telemetry.update();

        camera.initWebCamera();
        camera.initPipeline();
        camera.startCamera();

        camera.startCameraStream(0);
        CameraStreamServer.getInstance().setSource(camera.getCamera());

        telemetry.clear();
        telemetry.addLine("Camera Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            //camera.refreshScalars();

            telemetry.addData("Location", camera.sDeterminePosition());
            telemetry.update();
        }
    }
}