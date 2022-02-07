package org.firstinspires.ftc.teamcode.opmodes.Comp.old;

import static org.firstinspires.ftc.teamcode.hardware.lift.LIFT.*;
import static org.firstinspires.ftc.teamcode.util.Time.await;
import static org.firstinspires.ftc.teamcode.util.Time.timeout;

import androidx.core.view.WindowInsetsAnimationCompat;

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

import org.firstinspires.ftc.teamcode.vision.FreightFrenzyCamera;

import java.util.concurrent.atomic.AtomicReference;

@Disabled
@Autonomous(name = "All Auto(s)", group = "Comp")
public class Auto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        lift.LIFT pos = D1;

        Robot robot = new Robot(hardwareMap);
        CornettCore motionProfile = new CornettCore(robot);
        FreightFrenzyCamera camera = new FreightFrenzyCamera(hardwareMap);

        telemetry = robot.dashboard.getTelemetry();

        telemetry.addLine("Starting Hardware Map & Camera");
        telemetry.update();

        camera.initWebCamera();
        camera.initPipeline();
        camera.startCamera();

        camera.startCameraStream(0);
        CameraStreamServer.getInstance().setSource(camera.getCamera());

        robot.lift.primeServo();

        while(!isStarted()) {
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

        switch (camera.determinePosition()) {
            case A:
                pos = D1;
            case B:
                pos = D2;
            case C:
                pos = D3;
            default:
                pos = D1;
        }

        waitForStart();

        while(opModeIsActive()) {

            lift.LIFT finalPos1 = pos;
            new Thread(()-> {
                telemetry.addData("Stone Position", finalPos1.toString());
                telemetry.addData("Raw Position", camera.sDeterminePosition());
                telemetry.update();
            }).start();

        switch(MatchConfig.side) {
            case RED:
                switch(MatchConfig.park) {
                    case WAREHOUSE:
                        /*
                         * Init
                         */
                        robot.setSTART_POSITION(new Pose2D(63, 13, AngleUtil.interpretAngle(0)));

                        /*
                         * Run Auto
                        */

                        await(400, ()-> robot.intakeSys.regularFreightIntake());

                        motionProfile.runToPositionSync(40, 7, AngleUtil.interpretAngle(0), 1);
                        robot.DriveTrain.stopDrive();

                        motionProfile.rotateSync(Math.toRadians(45), Math.toRadians(5));

                        switch (pos) {
                            case D1:
                                robot.lift.setPosition(lift.liftOne);
                            case D2:
                                robot.lift.setPosition(lift.liftTwo);
                            case D3:
                                robot.lift.setPosition(lift.liftThree);
                        }

                        // Wait 5 Seconds -> for testing not running

                        sleep(5000);

                        boolean runTo = true;

                        if(runTo) {
                            motionProfile.runTimeSync(0.3, 300);
                        }

                        sleep(500);
                        robot.lift.drop();
                        sleep(1500);

                        robot.lift.primeServo();

                        await(500, ()-> {
                            robot.lift.startServo();
                            robot.lift.setPosition(lift.liftStart);
                        });

                        boolean split1 = false;

                        if(split1) {
                            // Tune
                            motionProfile.runToPositionSync(60, 7, Math.toRadians(45), 1);
                            robot.DriveTrain.stopDrive();

                            motionProfile.rotateSync(Math.toRadians(90), Math.toRadians(5));
                        }
                        else {
                            // Tune
                            motionProfile.runToPositionSync(60, 7, Math.toRadians(90), 3);
                        }


                        robot.DriveTrain.setMotorPowers(1, 0, 0);
                        sleep(1500);
                        robot.stopDrive();

                        robot.DriveTrain.setMotorPowers(0, 1, 0);
                        sleep(1500);
                        robot.stopDrive();

                        break;
                    case STORAGE:
                        /*
                         * Init
                         */
                        robot.setSTART_POSITION(new Pose2D(63, -36, AngleUtil.interpretAngle(0)));

                        /*
                         * Run Auto
                         */

                        AtomicReference<Double> position = new AtomicReference<>(lift.liftStart);

                        new Thread(()-> {
                            try {
                                robot.lift.setPosition(position.get());
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                        }).start();

                        await(200, ()-> robot.intakeSys.regularFreightIntake());

                        lift.LIFT finalPos = pos;
                        await(400, ()-> {
                            if(finalPos == D1) {
                                position.set((double)350);
                            }
                            else if(finalPos == D2) {
                                position.set((double)600);
                            }
                            else {
                                position.set((double)1000);
                            }
                        });

                        // Tune

                        motionProfile.runToPositionSync(36, -26, Math.toRadians(295),1);
                        robot.DriveTrain.stopDrive();

                        sleep(3000);
                        robot.lift.drop();
                        sleep(1500);

                        robot.lift.primeServo();
                        await(500, ()-> {
                            robot.lift.startServo();
                            position.set(lift.liftStart);
                        });

                        // Tune

                        motionProfile.runToPositionSync(46, -64, Math.toRadians(350), 1);
                        robot.intakeSys.raiseIntake();

                        robot.DriveTrain.setMotorPowers(-0.3, -0.3);
                        sleep(300);
                        robot.DriveTrain.stopDrive();

                        robot.getDuck().setPower(-0.5);
                        sleep(4000);
                        robot.getDuck().setPower(0.0);

                        // Tune
                        motionProfile.runToPositionSync(32, -56, Math.toRadians(0), 1);
                        robot.DriveTrain.stopDrive();

                        break;
                }
                break;
            case BLUE:
                switch(MatchConfig.park) {
                    case WAREHOUSE:
                        /*
                         * Init
                         */

                        robot.setSTART_POSITION(new Pose2D(-63, 13, AngleUtil.interpretAngle(180)));

                        /*
                         * Run Auto
                         */
                        break;

                    case STORAGE:
                        /*
                         * Init
                         */
                        robot.setSTART_POSITION(new Pose2D(-63, -36, AngleUtil.interpretAngle(180)));

                        /*
                         * Run Auto
                         */


                }
                break;
        }


            PoseStorage.autoEnd = robot.pos;
            camera.shutdownPipeline();

            stop();
        }
    }
}
