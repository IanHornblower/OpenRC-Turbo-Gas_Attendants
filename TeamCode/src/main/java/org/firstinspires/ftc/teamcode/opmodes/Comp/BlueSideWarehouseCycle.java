package org.firstinspires.ftc.teamcode.opmodes.Comp;

import static org.firstinspires.ftc.teamcode.util.Time.await;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.control.CornettCore;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.lift;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.Timer;
import org.firstinspires.ftc.teamcode.vision.FreightFrenzyCamera;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "2 Cycle Blue Cycle Auto", group = "Comp")
public class BlueSideWarehouseCycle extends LinearOpMode {

    enum ElementPosition {
        LEFT, MIDDLE, RIGHT, NONE
    }

    boolean primed = false;
    boolean ranTwice = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Starting Hardware Map & Camera");
        telemetry.update();

        MatchConfig.side = MatchConfig.SIDE.BLUE;  // Change for blue
        MatchConfig.park = MatchConfig.PARK.WAREHOUSE;

        // Init

        Robot robot = new Robot(hardwareMap);

        //VoltageSensor intakeVoltage = hardwareMap.voltageSensor.get("intake");

        CornettCore motionProfile = new CornettCore(robot);
        FreightFrenzyCamera camera = new FreightFrenzyCamera(hardwareMap);
        Timer runTime = new Timer();
        robot.lift.primeServo();

        robot.setSTART_POSITION(AutonomousConstants.BlueConstants.Warehouse.START_POSITION);

        // Set Camera Constants & then init

        FreightFrenzyCamera.abVerticalLine = 90; // Adjust
        FreightFrenzyCamera.bcVerticalLine = 300; // Adjust

        FreightFrenzyCamera.lowestBlobArea = 1500;  // Maybe make bigger idk

        camera.initWebCamera();
        camera.initPipeline();
        camera.startCamera();

        camera.startCameraStream(0);
        CameraStreamServer.getInstance().setSource(camera.getCamera());

        // Loop while waiting for Start

        robot.freightDetector.run();

        ElementPosition state = null;

        while (!isStarted()) {
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

        runTime.start();

        new Thread(()-> {
            telemetry.addData("Has block", robot.freightDetector.hasFreight());
            //telemetry.addData("Voltage Sensor", intakeVoltage.getVoltage());
            telemetry.update();
        }).start();;

        while (opModeIsActive() && !isStopRequested()) {

            /*
             * Run Auto -- SPLIT CODE LATER
             * TODO: adjust angles, for 1 & 2 cycles and test all auto types
             */

            await(200, () -> robot.intakeSys.regularFreightIntake());

            switch (state) {
                case LEFT:
                    await(400, () -> robot.lift.SyncSetPosition(lift.liftOne));
                    motionProfile.runToPositionSync(AutonomousConstants.BlueConstants.Warehouse.SHIPPING_HUB_LEVEL_ONE, 1300, 1);
                    break;
                case MIDDLE:
                    await(400, () -> robot.lift.SyncSetPosition(lift.liftTwo));
                    motionProfile.runToPositionSync(AutonomousConstants.BlueConstants.Warehouse.SHIPPING_HUB_LEVEL_TWO, 1300, 1);
                    break;
                case RIGHT:
                    await(400, () -> robot.lift.SyncSetPosition(lift.liftThree));
                    motionProfile.runToPositionSync(AutonomousConstants.BlueConstants.Warehouse.SHIPPING_HUB_LEVEL_THREE, 1300, 1);
                    break;
                default:
                    await(400, () -> robot.lift.SyncSetPosition(lift.liftOne));
                    motionProfile.runToPositionSync(AutonomousConstants.BlueConstants.Warehouse.SHIPPING_HUB_LEVEL_THREE, 1300, 1);
                    break;
            }

            robot.lift.drop();
            sleep(1500);
            robot.lift.primeServo();

            await(300, () -> robot.lift.startServo());
            robot.lift.setPosition(lift.liftStart);

            primed = false;
            ranTwice = false;

            // Cycle 1 //
            // Run into wall
            motionProfile.runToPositionSync(AutonomousConstants.BlueConstants.Warehouse.WAREHOUSE_WALL, 1000, 1);
            runInRelation(robot, -1, 0, 0, 500);

            // Start Intake Loop
            robot.intakeSys.setIntakePower(1.0);  // Start intaking

            double currentY = robot.pos.y;
            while (robot.pos.y < currentY + AutonomousConstants.BlueConstants.Warehouse.DISTANCE_BLIND) {  // Drive for certain distance
                robot.updateOdometry();
                robot.DriveTrain.setMotorPowers(-0.1, 1, 0);
            }
            robot.stopDrive();

            intakeLoop(robot);  // Intake stuff

            while (robot.pos.y > AutonomousConstants.BlueConstants.Warehouse.RETURN_DISTANCE_MIN) {
                robot.updateOdometry();
                robot.DriveTrain.setMotorPowers(-0.1, -1, 0);
            }
            robot.stopDrive();

            robot.intakeSys.setIntakePower(0.0);

            await(400, () -> robot.lift.SyncSetPosition(lift.liftThree));

            motionProfile.runToPositionSync(AutonomousConstants.BlueConstants.Warehouse.SHIPPING_HUB_LEVEL_CYCLE_1, 1300, 1);

            robot.lift.drop();
            sleep(2000);

            robot.lift.primeServo();
            await(300, () -> robot.lift.startServo());

            robot.lift.setPosition(lift.liftStart);

            primed = false;
            ranTwice = false;

            // Loop Again // Cycle 2
            // Start
            motionProfile.runToPositionSync(new Pose2D(-64.5, 3, Math.toRadians(80)), 1500, 1);

            runInRelation(robot, -0.6, 0, 0, 800);

            // Start Intake Loop8
            robot.intakeSys.setIntakePower(1.0);

            currentY = robot.pos.y;

            while (robot.pos.y < currentY + AutonomousConstants.BlueConstants.Warehouse.DISTANCE_BLIND) {
                robot.updateOdometry();
                robot.DriveTrain.setMotorPowers(-0.1, 1, 0);
            }
            robot.stopDrive();

            intakeLoop(robot); // yup

            while (robot.pos.y > AutonomousConstants.BlueConstants.Warehouse.RETURN_DISTANCE_MIN) {
                robot.updateOdometry();
                robot.DriveTrain.setMotorPowers(-0.1, -1, 0);
            }
            robot.stopDrive();

            robot.intakeSys.setIntakePower(0.0);

            // End Loop
            await(400, () -> robot.lift.SyncSetPosition(lift.liftThree));

            motionProfile.runToPositionSync(AutonomousConstants.BlueConstants.Warehouse.SHIPPING_HUB_LEVEL_CYCLE_2, 1300, 1);

            robot.lift.drop();
            sleep(2000);

            robot.lift.primeServo();
            await(300, () -> robot.lift.startServo());

            robot.lift.setPosition(lift.liftStart);

            //////////////// PARK /////////////

            motionProfile.runToPositionSync(new Pose2D(-64.5, -8, Math.toRadians(80)), 1000, 0.1);

            runInRelation(robot, -1, 0, 0, 300);

            while (robot.pos.y < AutonomousConstants.BlueConstants.Warehouse.PARK_DISTANCE) {
                robot.updateOdometry();
                robot.DriveTrain.setMotorPowers(-0.085, 1, 0);
            }
            robot.DriveTrain.stopDrive();

            PoseStorage.autoEnd = robot.pos;

            camera.shutdownPipeline();
            stop();
        }


    }

    public void runInRelation(Robot robot, double x, double y, double t, double time) {
        Timing.Timer timer = new Timing.Timer((long) time, TimeUnit.MILLISECONDS);
        timer.start();
        while (!timer.done()) {
            robot.updateOdometry();
            robot.DriveTrain.setMotorPowers(x, y, t);
        }
        robot.stopDrive();
    }

    public void primeLift(Robot robot) throws InterruptedException {
        primed = true;
        robot.lift.primeServo();
        robot.lift.setPosition(lift.liftOne);
    }

    public void retractLift(Robot robot) throws InterruptedException {
        primed = false;
        robot.lift.startServo();
        robot.lift.setPosition(lift.liftStart);
    }

    public void intakeLoop(Robot robot) throws InterruptedException {
        while (!robot.freightDetector.hasFreight()) {  // Drive forward till freight
            robot.updateOdometry();
            robot.DriveTrain.setMotorPowers(0, 0.5, 0);
        }
        robot.DriveTrain.stopDrive();

        runInRelation(robot, -0.6, -0.5, 0, 800);
        robot.intakeSys.setIntakePower(0.0);
        primeLift(robot);  // Ready lift

        while (!robot.freightDetector.hasFreight()) {  // Drive forward till freight
            ranTwice = true;
            robot.updateOdometry();

            robot.intakeSys.setIntakePower(1.0);
            robot.DriveTrain.setMotorPowers(0, 0.5, 0);

            if(primed) {
                retractLift(robot); // If lift fucks up this is why
            }
        }
        robot.DriveTrain.stopDrive();

        if(ranTwice) {
            runInRelation(robot, -0.6, -0.5, 0, 500);
            robot.intakeSys.setIntakePower(0.0);

            primeLift(robot);  // Ready lift
        }

        robot.intakeSys.setIntakePower(-0.9);
        robot.lift.setPosition(lift.liftOne);
    }

}