package org.firstinspires.ftc.teamcode.opmodes.Comp;

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

    @Override
    public void runOpMode() throws InterruptedException {
        lift.LIFT pos = D1;

        Robot robot = new Robot(hardwareMap);
        CornettCore motionProfile = new CornettCore(robot);
        FreightFrenzyCamera camera = new FreightFrenzyCamera(hardwareMap);

        robot.setSTART_POSITION(new Pose2D(63, 13, AngleUtil.interpretAngle(0)));

        telemetry.addLine("Starting Hardware Map & Camera");
        telemetry.update();

        camera.initWebCamera();
        camera.initPipeline();
        camera.startCamera();

        camera.startCameraStream(0);
        CameraStreamServer.getInstance().setSource(camera.getCamera());

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
             * Run Auto
             */

            await(200, ()-> robot.intakeSys.regularFreightIntake());

            //Trajectory toPlate = new Trajectory(robot, robot.START_POSITION);

            //toPlate.addWaypoint(new Point(55.333333333333336, 13.11111111111111));
            //toPlate.addWaypoint(new Point(48, -5));
            //toPlate.addWaypoint(new Point(47, -5));

            //toPlate.followPath(Trajectory.PATH_TYPE.PURE_PURSUIT, CornettCore.DIRECTION.BACKWARD, 6, 1);

            Timer plate = new Timer();
            plate.start();
            while(plate.currentSeconds() < 2.0) {
                motionProfile.runToPositionSync(45, -5, Math.toRadians(15), 1);
            }

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
            await(300, ()->robot.lift.startServo());

            robot.lift.setPosition(lift.liftStart);

            // Start

            motionProfile.runToPositionSync(62, 7, Math.toRadians(90), 1);

            robot.DriveTrain.setMotorPowers(1, 0, 0);
            sleep(500);
            robot.stopDrive();

            robot.intakeSys.setIntakePower(-0.8);

            Pose2D run1 = robot.pos;

            Trajectory run1Traj = new Trajectory(robot, run1);
            run1Traj.addWaypoint(new Point(run1.x+5, run1.y+40));

            run1Traj.followPath(Trajectory.PATH_TYPE.PURE_PURSUIT, CornettCore.DIRECTION.FORWARD, 8, 1);



            robot.intakeSys.setIntakePower(0.0);

            stop();
        }
    }
}
