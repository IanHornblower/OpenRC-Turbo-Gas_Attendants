package org.firstinspires.ftc.teamcode.opmodes.Auto.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.teamcode.hardware.FreightDetector;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.opmodes.AutoConstants.CarouselAuto;
import org.firstinspires.ftc.teamcode.opmodes.AutoConstants.PlaceParkAuto;
import org.firstinspires.ftc.teamcode.opmodes.AutoConstants.WarehouseCycleAuto;
import org.firstinspires.ftc.teamcode.opmodes.MatchConfig;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.FreightFrenzyCamera;

@Autonomous
public class CameraTest extends LinearOpMode {

    enum ElementPosition {
        LEFT, MIDDLE, RIGHT, NONE
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Starting Hardware Map & Camera");
        telemetry.update();

        Robot robot = new Robot(hardwareMap);
        robot.driveController.setGamepad(gamepad1);
        robot.operatorController.setGamepad(gamepad2);

        FreightFrenzyCamera camera = new FreightFrenzyCamera(hardwareMap);

        robot.driveTrain.setPoseEstimate(PlaceParkAuto.RedConstants.START_POSITION);

        camera.initWebCamera();
        camera.initPipeline();
        camera.startCamera();

        camera.startCameraStream(0);
        CameraStreamServer.getInstance().setSource(camera.getCamera());

        ElementPosition state = null;

        while(!opModeIsActive() && !isStopRequested()) {
            switch (camera.determinePosition()) {
                case A:
                    state = ElementPosition.LEFT;
                    break;
                case B:
                    state = ElementPosition.MIDDLE;
                    break;
                case C:
                    state = ElementPosition.RIGHT;
                    break;
                case ABSENT: // Account for displacement
                    state = ElementPosition.RIGHT;
                    break;
            }

            telemetry.clear();
            telemetry.addLine("Waiting For Start");
            telemetry.addLine(
                    "Autonomous Configuration: \n" +
                            "Side: " + MatchConfig.side.toString() +
                            "\nParking Location: " + MatchConfig.park.toString());
            telemetry.addData("Location", camera.sDeterminePosition());
            telemetry.update();
        }


        telemetry.clear();
        telemetry.update();
        waitForStart();
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && !isStopRequested()) {
            sleep(10000);
            telemetry.addLine("DONE");
            telemetry.update();
            stop();

        }
    }
}
