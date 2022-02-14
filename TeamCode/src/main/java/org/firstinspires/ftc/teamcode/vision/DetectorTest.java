package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera2;

@TeleOp(name="Detector Test", group="test")
public class DetectorTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        Detector detector = new Detector(hardwareMap);
        telemetry.addData("initializing", "done");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {


            FtcDashboard.getInstance().startCameraStream(OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, 1), 0);

            if (gamepad1.a) {
                detector.loadImage();
            }
            // Default position is NONE so make sure you click 'A' to get the actual position.
            Detector.ElementPosition pos = detector.getElementPosition();
            telemetry.addData("seen objects", pos == Detector.ElementPosition.LEFT ? "left" :
                    (pos == Detector.ElementPosition.RIGHT ? "right" : "none"));
            telemetry.update();
        }
    }
}
