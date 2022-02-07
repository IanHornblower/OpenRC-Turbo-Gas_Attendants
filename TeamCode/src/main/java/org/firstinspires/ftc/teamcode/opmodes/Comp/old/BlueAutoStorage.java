package org.firstinspires.ftc.teamcode.opmodes.Comp.old;

import static org.firstinspires.ftc.teamcode.hardware.lift.LIFT.D1;
import static org.firstinspires.ftc.teamcode.hardware.lift.LIFT.D2;
import static org.firstinspires.ftc.teamcode.hardware.lift.LIFT.D3;
import static org.firstinspires.ftc.teamcode.util.Time.await;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.control.CornettCore;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.lift;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.opmodes.Comp.MatchConfig;
import org.firstinspires.ftc.teamcode.util.AngleUtil;
import org.firstinspires.ftc.teamcode.util.Time;
import org.firstinspires.ftc.teamcode.vision.FreightFrenzyCamera;

import java.util.concurrent.atomic.AtomicReference;

@Disabled
@Autonomous(name = "Blue Auto Storage", group = "Comp")
public class BlueAutoStorage extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        lift.LIFT pos = D1;

        Robot robot = new Robot(hardwareMap);
        CornettCore motionProfile = new CornettCore(robot);
        FreightFrenzyCamera camera = new FreightFrenzyCamera(hardwareMap);

        telemetry.addLine("Starting Hardware Map & Camera");
        telemetry.update();

        camera.initWebCamera();
        camera.initPipeline();
        camera.startCamera();

        //camera.startCameraStream(0);
        //CameraStreamServer.getInstance().setSource(camera.getCamera());

        robot.lift.primeServo();

        while(!isStarted()) {
            switch (camera.determinePosition()) {
                case A:
                    pos = D1;
                case B:
                    pos = D2;
                case C:
                    pos = D3;
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

        camera.shutdownPipeline();

        while(opModeIsActive() && !isStopRequested()) {
            /*
             * Init
             */
            robot.setSTART_POSITION(new Pose2D(-63, -36, AngleUtil.interpretAngle(180)));

            /*
             * Run Auto
             */

            await(200, ()-> robot.intakeSys.regularFreightIntake());
            // Tune

            motionProfile.runToPositionSync(-38, -28, Math.toRadians(235),1);  // Tune
            robot.DriveTrain.stopDrive();

            switch (camera.determinePosition()) {
                case A:
                    robot.lift.SyncSetPosition(350);
                    break;
                case B:
                    robot.lift.SyncSetPosition(600);
                    break;
                case C:
                    robot.lift.SyncSetPosition(1050);
                    break;
            }

            robot.lift.drop();

            sleep(1500);

            robot.lift.primeServo();

            sleep(300);

            robot.lift.startServo();
            robot.lift.setPosition(lift.liftStart);

            sleep(100);

            // Tune

            motionProfile.runToPositionSync(-46, -60, Math.toRadians(190), 1);
            robot.intakeSys.raiseIntake();

            robot.DriveTrain.setMotorPowers(-0.3, -0.3);
            sleep(400);
            robot.DriveTrain.stopDrive();

            robot.getDuck().setPower(0.4);
            sleep(5000);
            robot.getDuck().setPower(0.0);

            // Tune
            motionProfile.runToPositionSync(-37, -56, Math.toRadians(180), 2);
            robot.DriveTrain.stopDrive();

            PoseStorage.autoEnd = robot.pos;

            stop();
        }
    }
}
