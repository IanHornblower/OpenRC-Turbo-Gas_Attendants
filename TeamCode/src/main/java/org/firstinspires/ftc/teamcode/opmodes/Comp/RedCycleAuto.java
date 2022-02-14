package org.firstinspires.ftc.teamcode.opmodes.Comp;

import static org.firstinspires.ftc.teamcode.hardware.lift.LIFT.D1;
import static org.firstinspires.ftc.teamcode.hardware.lift.LIFT.D2;
import static org.firstinspires.ftc.teamcode.hardware.lift.LIFT.D3;
import static org.firstinspires.ftc.teamcode.util.Time.await;
import static org.firstinspires.ftc.teamcode.util.Time.timeout;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.checkerframework.checker.units.UnitsTools;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.control.CornettCore;
import org.firstinspires.ftc.teamcode.control.Trajectory;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.lift;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.opmodes.Comp.MatchConfig;
import org.firstinspires.ftc.teamcode.util.AngleUtil;
import org.firstinspires.ftc.teamcode.util.Timer;
import org.firstinspires.ftc.teamcode.vision.FreightFrenzyCamera;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Red Cycle Auto", group = "Comp")
public class RedCycleAuto extends LinearOpMode {

    enum ElementPosition {
        LEFT, MIDDLE, RIGHT, NONE
    }

    boolean runTwice = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Starting Hardware Map & Camera");
        telemetry.update();

        // Init

        Robot robot = new Robot(hardwareMap);
        CornettCore motionProfile = new CornettCore(robot);
        FreightFrenzyCamera camera = new FreightFrenzyCamera(hardwareMap);
        Timer runTime = new Timer();
        robot.lift.primeServo();

        robot.setSTART_POSITION(AutonomousConstants.RedConstants.Warehouse.START_POSITION);

        camera.initWebCamera();
        camera.initPipeline();
        camera.startCamera();

        camera.startCameraStream(0);
        CameraStreamServer.getInstance().setSource(camera.getCamera());

        // Loop while waiting for Start

        ElementPosition state = null;

        while(!isStarted()) {
            switch (camera.determinePosition()) {
                case A:
                    state = ElementPosition.LEFT;
                case B:
                    state = ElementPosition.MIDDLE;
                case C:
                    state = ElementPosition.RIGHT;
                default:
                    state = ElementPosition.MIDDLE;  // May Switch later this is the easiest
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

        runTime.start();

        while(opModeIsActive() && !isStopRequested()) {

            // TESTING LEFT
            state = ElementPosition.LEFT;

            /*
             * Run Auto -- SPLIT CODE LATER
             */

            await(200, ()-> robot.intakeSys.regularFreightIntake());

            await(400, ()->robot.lift.SyncSetPosition(lift.liftOne));

            //Timing.Timer joe = new Timing.Timer(2);
            //joe.start();

            motionProfile.runToPositionSync(AutonomousConstants.RedConstants.Warehouse.SHIPPING_HUB_LEVEL_ONE, 1300, 1);

            robot.lift.drop();
            sleep(1500);

            robot.lift.primeServo();

            if(robot.freightDetector.hasFreight()) { // If the block gets stuck drop it again
                runTwice = true;
                robot.lift.drop();
                sleep(2000);

                robot.lift.primeServo();
            }

            await(300, ()->robot.lift.startServo());

            robot.lift.setPosition(lift.liftStart);

            // Cycle 1 //

            // Run into wall
            motionProfile.runToPositionSync(AutonomousConstants.RedConstants.Warehouse.WAREHOUSE_WALL, 1000, 1);

            runInRelation(robot, 1, 0, 0, 500);

            // Start Intake Loop

            robot.intakeSys.setIntakePower(0.9);

            double currentY = robot.pos.y;

            while(robot.pos.y < currentY + AutonomousConstants.RedConstants.Warehouse.DISTANCE_BLIND) {
                robot.updateOdometry();
                robot.DriveTrain.setMotorPowers(0.3, 1, 0);
            }
            robot.stopDrive();

            while(!robot.freightDetector.hasFreight()) {
                robot.updateOdometry();
                robot.DriveTrain.setMotorPowers(0, 0.5, 0);
            }
            robot.DriveTrain.stopDrive();

            robot.lift.primeServo();

            robot.intakeSys.setIntakePower(0.0);

            runInRelation(robot, 0.6, 0, 0, 800);

            await(200, ()-> robot.intakeSys.setIntakePower(-0.9));

            robot.lift.setPosition(lift.liftOne);

            while(robot.pos.y > AutonomousConstants.RedConstants.Warehouse.RETURN_DISTANCE_MIN) {
                robot.updateOdometry();
                robot.DriveTrain.setMotorPowers(0.3 , -1, 0);
            }
            robot.stopDrive();

            robot.intakeSys.setIntakePower(0.0);

            // End Loop

            //motionProfile.runToPositionSync(AutonomousConstants.RedConstants.Warehouse.WAREHOUSE_WALL, 1);

            await(400, ()->robot.lift.SyncSetPosition(lift.liftThree));

            motionProfile.runToPositionSync(AutonomousConstants.RedConstants.Warehouse.SHIPPING_HUB_LEVEL_THREE, 1300, 1);

            robot.lift.drop();
            sleep(1500);

            robot.lift.primeServo();
            await(300, ()->robot.lift.startServo());

            robot.lift.setPosition(lift.liftStart);

            // Loop Again

            // Start
            motionProfile.runToPositionSync(new Pose2D(64.5, 3, Math.toRadians(90)), 1500, 1);

            runInRelation(robot, 1, 0, 0, 800);

            // Start Intake Loop

            robot.intakeSys.setIntakePower(0.9);

            currentY = robot.pos.y;

            while(robot.pos.y < currentY + AutonomousConstants.RedConstants.Warehouse.DISTANCE_BLIND) {
                robot.updateOdometry();
                robot.DriveTrain.setMotorPowers(0.3, 1, 0);
            }
            robot.stopDrive();

            while(!robot.freightDetector.hasFreight()) {
                robot.updateOdometry();
                robot.DriveTrain.setMotorPowers(0, 0.5, 0);
            }
            robot.DriveTrain.stopDrive();

            robot.lift.primeServo();

            robot.intakeSys.setIntakePower(0.0);

            runInRelation(robot, 0.6, 0, 0, 800);

            await(200, ()-> robot.intakeSys.setIntakePower(-0.9));

            robot.lift.setPosition(lift.liftOne);

            while(robot.pos.y > AutonomousConstants.RedConstants.Warehouse.RETURN_DISTANCE_MIN) {
                robot.updateOdometry();
                robot.DriveTrain.setMotorPowers(0.3 , -1, 0);
            }
            robot.stopDrive();

            robot.intakeSys.setIntakePower(0.0);

            // End Loop

            //motionProfile.runToPositionSync(AutonomousConstants.RedConstants.Warehouse.WAREHOUSE_WALL, 1);

            await(400, ()->robot.lift.SyncSetPosition(lift.liftThree));

            motionProfile.runToPositionSync(AutonomousConstants.RedConstants.Warehouse.SHIPPING_HUB_LEVEL_THREE, 1300, 1);

            robot.lift.drop();
            sleep(1500);

            robot.lift.primeServo();
            await(300, ()->robot.lift.startServo());

            robot.lift.setPosition(lift.liftStart);

            if(runTime.currentSeconds() > 22) {
                runTwice = true;
            }

            if(!runTwice) {
                // Loop Again 3rd Time //

                // Start
                motionProfile.runToPositionSync(new Pose2D(64.5, 3, Math.toRadians(90)), 1500, 1);

                runInRelation(robot, 1, 0, 0, 800);

                // Start Intake Loop

                robot.intakeSys.setIntakePower(0.9);

                currentY = robot.pos.y;

                while(robot.pos.y < currentY + AutonomousConstants.RedConstants.Warehouse.DISTANCE_BLIND) {
                    robot.updateOdometry();
                    robot.DriveTrain.setMotorPowers(0.3, 1, 0);
                }
                robot.stopDrive();

                while(!robot.freightDetector.hasFreight()) {
                    robot.updateOdometry();
                    robot.DriveTrain.setMotorPowers(0, 0.3, 0);
                }
                robot.DriveTrain.stopDrive();

                robot.lift.primeServo();

                robot.intakeSys.setIntakePower(0.0);

                runInRelation(robot, 0.6, 0, 0, 800);

                await(200, ()-> robot.intakeSys.setIntakePower(-0.9));

                robot.lift.setPosition(lift.liftOne);

                while(robot.pos.y > AutonomousConstants.RedConstants.Warehouse.RETURN_DISTANCE_MIN) {
                    robot.updateOdometry();
                    robot.DriveTrain.setMotorPowers(0.3 , -1, 0);
                }
                robot.stopDrive();

                robot.intakeSys.setIntakePower(0.0);

                // End Loop

                //motionProfile.runToPositionSync(AutonomousConstants.RedConstants.Warehouse.WAREHOUSE_WALL, 1);

                await(400, ()->robot.lift.SyncSetPosition(lift.liftThree));

                motionProfile.runToPositionSync(new Pose2D(44, -5, Math.toRadians(5)), 1500, 1);

                robot.lift.drop();
                sleep(1500);

                robot.lift.primeServo();
                await(300, ()->robot.lift.startServo());

                robot.lift.setPosition(lift.liftStart);
            }

            //////////////// PARK /////////////

            motionProfile.runToPositionSync(new Pose2D(65.5, 0, Math.toRadians(100)), 800, 0.1);

            runInRelation(robot, 1, 0, 0, 300);

            currentY = robot.pos.y;

            while(robot.pos.y < currentY + AutonomousConstants.RedConstants.Warehouse.DISTANCE_BLIND-10) {
                robot.updateOdometry();
                robot.DriveTrain.setMotorPowers(0.3, 1, 0);
            }
            robot.DriveTrain.stopDrive();

            camera.shutdownPipeline();
            stop();
        }


    }

    public void runInRelation(Robot robot, double x, double y, double t, double time) {
        Timing.Timer timer = new Timing.Timer((long)time, TimeUnit.MILLISECONDS);
        timer.start();
        while(!timer.done()) {
            robot.updateOdometry();
            robot.DriveTrain.setMotorPowers(x, y, t);
        }
        robot.stopDrive();
    }
}