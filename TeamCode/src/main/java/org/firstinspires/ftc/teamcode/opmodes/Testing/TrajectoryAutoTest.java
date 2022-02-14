package org.firstinspires.ftc.teamcode.opmodes.Testing;

import static android.os.SystemClock.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Iterative;
import org.firstinspires.ftc.teamcode.control.*;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.AngleUtil;

import java.util.ArrayList;
@Config
@Autonomous(name = "Motion Profile Testing", group = "Traj")
public class TrajectoryAutoTest extends LinearOpMode {

    public static boolean turn = false;
    public static boolean loop = false;
    public static double interval = 0;
    public static double angle = 90;


    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        CornettCore motionProfile = new CornettCore(robot);

        robot.setSTART_POSITION(new Pose2D(0, 0, AngleUtil.interpretAngle(90)));

        // Create Trajectories and Function Lists

        robot.intakeSys.regularFreightIntake();
        robot.lift.startServo();

        Trajectory joe = new Trajectory(robot, robot.START_POSITION);

        joe.addWaypoint(new Point(0, 24));
        joe.addWaypoint(new Point(24, 48));
        //joe.addWaypoint(new Point(0, 0));

        Trajectory backJoe = new Trajectory(robot, joe.end());

        backJoe.addWaypoint(new Point(0, 24));
        backJoe.addWaypoint(new Point(0, 0));

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            //motionProfile.runToPositionSync(24, 24, 0, 1);

            //joe.followPath(Trajectory.PATH_TYPE.PURE_PURSUIT, CornettCore.DIRECTION.FORWARD, 12, 1);

            //sleep(1000);

            //backJoe.followPath(Trajectory.PATH_TYPE.PURE_PURSUIT, CornettCore.DIRECTION.BACKWARD, 12, 1);

            motionProfile.runToPositionSync(100, 100, Math.toRadians(90), 500, 1);




            stop();

        }

    }
}
