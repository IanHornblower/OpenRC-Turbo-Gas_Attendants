package org.firstinspires.ftc.teamcode.opmodes.Comp.old;

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

import static org.firstinspires.ftc.teamcode.hardware.lift.LIFT.*;
import static org.firstinspires.ftc.teamcode.util.Time.await;

@Disabled
@Autonomous(name = "All Auto(s)", group = "Comp")
public class Auto_Depricated extends LinearOpMode {

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

        while(opModeIsActive()) {

        switch(MatchConfig.side) {
            case RED:
                switch(MatchConfig.park) {
                    case WAREHOUSE:
                        /*
                         * Init
                         */
                        robot.lift.startServo();
                        robot.setSTART_POSITION(new Pose2D(63, 13, AngleUtil.interpretAngle(0)));

                        /*
                         * Run Auto
                        */

                        await(400, ()-> robot.intakeSys.regularFreightIntake());

                        motionProfile.runToPositionSync(40, 7, Math.toRadians(45), 1);

                        robot.DriveTrain.stopDrive();

                        switch (pos) {
                            case D1:
                                robot.lift.setPosition(lift.liftOne);
                            case D2:
                                robot.lift.setPosition(lift.liftTwo);
                            case D3:
                                robot.lift.setPosition(lift.liftThree);
                        }

                        sleep(500);
                        robot.lift.drop();
                        sleep(1500);

                        robot.lift.primeServo();

                        await(500, ()-> {
                            robot.lift.startServo();
                            robot.lift.setPosition(lift.liftStart);
                        });

                        motionProfile.runToPositionSync(60, 7, Math.toRadians(90), 3);

                        robot.DriveTrain.stopDrive();

                        robot.DriveTrain.setMotorPowers(1, 0, 0);
                        sleep(1500);
                        robot.stopDrive();

                        robot.DriveTrain.setMotorPowers(0, 1, 0);
                        sleep(2000);
                        robot.stopDrive();

                        stop();

                        break;
                    case STORAGE:
                        /*
                         * Init
                         */
                        robot.setSTART_POSITION(new Pose2D(63, -36, AngleUtil.interpretAngle(0)));

                        /*
                         * Run Auto
                         */

                        await(400, ()-> robot.intakeSys.regularFreightIntake());

                        motionProfile.runToPositionSync(38, -28, AngleUtil.interpretAngle(0),1);
                        robot.DriveTrain.stopDrive();
                        motionProfile.rotateSync(Math.toRadians(300), Math.toRadians(5));

                        //switch (pos) {
                        //    case D1:
                        //        robot.lift.setPosition(lift.liftOne);
                        //    case D2:
                        //        robot.lift.setPosition(lift.liftTwo);
                        //    case D3:
                        //        robot.lift.setPosition(lift.liftThree);
                        //}

                        //sleep(500);
                        //robot.lift.drop();
                        //sleep(1500);

                        //robot.lift.primeServo();

                        //await(500, ()-> {
                        //    robot.lift.startServo();
                        //    robot.lift.setPosition(lift.liftStart);
                        //});

                        motionProfile.runToPositionSync(50, -62, Math.toRadians(0), 1);
                        robot.DriveTrain.stopDrive();

                        robot.DriveTrain.setMotorPowers(-0.2, -0.3);
                        sleep(400);
                        robot.DriveTrain.stopDrive();

                        sleep(500);
                        robot.getDuck().setPower(-0.5);
                        sleep(3000);
                        robot.getDuck().setPower(0.0);

                        robot.intakeSys.raiseIntake();

                        motionProfile.runToPositionSync(36, -59, Math.toRadians(0), 2);
                        robot.DriveTrain.stopDrive();

                        stop();
                        break;
                }
                break;
            case BLUE:
                switch(MatchConfig.park) {
                    case WAREHOUSE:
                        /*
                         * Init
                         */
                        robot.lift.startServo();
                        robot.setSTART_POSITION(new Pose2D(-63, 13, AngleUtil.interpretAngle(180)));

                        /*
                         * Run Auto
                         */

                        await(400, ()-> robot.intakeSys.regularFreightIntake());

                        motionProfile.runToPositionSync(-40, 7, Math.toRadians(135), 1);

                        robot.DriveTrain.stopDrive();

                        switch (pos) {
                            case D1:
                                robot.lift.setPosition(lift.liftOne);
                            case D2:
                                robot.lift.setPosition(lift.liftTwo);
                            case D3:
                                robot.lift.setPosition(lift.liftThree);
                        }

                        sleep(500);
                        robot.lift.drop();
                        sleep(1500);

                        robot.lift.primeServo();

                        await(500, ()-> {
                            robot.lift.startServo();
                            robot.lift.setPosition(lift.liftStart);
                        });

                        motionProfile.runToPositionSync(-60, 7, Math.toRadians(90), 3);

                        robot.DriveTrain.setMotorPowers(-1, 0, 0);
                        sleep(1500);
                        robot.stopDrive();

                        robot.DriveTrain.setMotorPowers(0, 1, 0);
                        sleep(1500);

                        robot.stopDrive();
                        stop();
                        break;
                    case STORAGE:
                        /*
                         * Init
                         */
                        robot.lift.startServo();
                        robot.setSTART_POSITION(new Pose2D(-63, -36, AngleUtil.interpretAngle(180)));

                        /*
                         * Run Auto
                         */

                        await(400, ()-> robot.intakeSys.regularFreightIntake());

                        motionProfile.runToPositionSync(-40, -30, Math.toRadians(225),1);

                        robot.DriveTrain.stopDrive();

                        switch (pos) {
                            case D1:
                                robot.lift.setPosition(lift.liftOne);
                            case D2:
                                robot.lift.setPosition(lift.liftTwo);
                            case D3:
                                robot.lift.setPosition(lift.liftThree);
                        }

                        sleep(500);
                        robot.lift.drop();
                        sleep(1500);

                        robot.lift.primeServo();

                        await(500, ()-> {
                                    robot.lift.startServo();
                                    robot.lift.setPosition(lift.liftStart);
                                });

                        motionProfile.runToPositionSync(-50, -62, Math.toRadians(180), 1);
                        robot.DriveTrain.stopDrive();

                        robot.DriveTrain.setMotorPowers(-0.2, -0.3);
                        sleep(400);
                        robot.DriveTrain.stopDrive();

                        sleep(500);
                        robot.getDuck().setPower(0.5);
                        sleep(3000);
                        robot.getDuck().setPower(0.0);

                        robot.intakeSys.raiseIntake();

                        motionProfile.runToPositionSync(-36, -59, Math.toRadians(180), 2);
                        robot.DriveTrain.stopDrive();
                        stop();

                        break;
                }
                break;
        }


            PoseStorage.autoEnd = robot.pos;
            camera.shutdownPipeline();

            stop();
        }
    }
}
