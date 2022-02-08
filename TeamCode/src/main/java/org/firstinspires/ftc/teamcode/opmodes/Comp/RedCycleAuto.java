package org.firstinspires.ftc.teamcode.opmodes.Comp;

import static org.firstinspires.ftc.teamcode.hardware.lift.LIFT.D1;
import static org.firstinspires.ftc.teamcode.hardware.lift.LIFT.D2;
import static org.firstinspires.ftc.teamcode.hardware.lift.LIFT.D3;
import static org.firstinspires.ftc.teamcode.util.Time.await;
import static org.firstinspires.ftc.teamcode.util.Time.timeout;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

@Autonomous(name = "Red Cycle Auto", group = "Comp")
public class RedCycleAuto extends LinearOpMode {

    enum ElementPosition {
        LEFT, MIDDLE, RIGHT, NONE
    }

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

            //await(400, ()->robot.lift.SyncSetPosition(lift.liftOne));
            motionProfile.runToPositionSync(AutonomousConstants.RedConstants.Warehouse.SHIPPING_HUB_LEVEL_ONE, 1);


            //robot.lift.drop();
            //sleep(1500);

            //robot.lift.primeServo();
            //await(300, ()->robot.lift.startServo());

            //robot.lift.setPosition(lift.liftStart);



            // Start



            motionProfile.runToPositionSync(AutonomousConstants.RedConstants.Warehouse.WAREHOUSE_WALL, 1);

            robot.DriveTrain.setMotorPowers(1, 0, 0);
            sleep(200);
            robot.stopDrive();

            robot.intakeSys.setIntakePower(0.9);

            while(!robot.freightDetector.hasFreight()) {
                robot.DriveTrain.setMotorPowers(0, 0.7, 0);
            }
            robot.DriveTrain.stopDrive();

            motionProfile.runToPositionSync(AutonomousConstants.RedConstants.Warehouse.WAREHOUSE_WALL, 1);

            motionProfile.runToPositionSync(AutonomousConstants.RedConstants.Warehouse.SHIPPING_HUB_LEVEL_THREE, 1);

            /*

            Pose2D run1 = robot.pos;

            Trajectory run1Traj = new Trajectory(robot, run1);
            run1Traj.addWaypoint(new Point(run1.x+5, run1.y+40));

            run1Traj.followPath(Trajectory.PATH_TYPE.PURE_PURSUIT, CornettCore.DIRECTION.FORWARD, 8, 1);



            robot.intakeSys.setIntakePower(0.0);

            // Done


             */
            camera.shutdownPipeline();
            stop();
        }
    }
}
