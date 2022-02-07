package org.firstinspires.ftc.teamcode.opmodes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.control.CornettCore;
import org.firstinspires.ftc.teamcode.control.Trajectory;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.Curve;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.AngleUtil;
import org.firstinspires.ftc.teamcode.util.Array;
import org.firstinspires.ftc.teamcode.util.Controller;
import static org.firstinspires.ftc.teamcode.util.Controller.*;
import static org.firstinspires.ftc.teamcode.util.MathUtil.roundPlaces;

import org.firstinspires.ftc.teamcode.util.MathUtil;

@Disabled
@Autonomous(name = "Testing Auto", group = "Testing")
public class TestAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        robot.setSTART_POSITION(new Pose2D(0, 0, AngleUtil.interpretAngle(90)));

        waitForStart();

        while(opModeIsActive()) {
            robot.updateOdometry();

            CornettCore motionProfile = new CornettCore(robot);
            Trajectory path1 = new Trajectory(robot, robot.START_POSITION);

            path1.addWaypoint(new Pose2D(0, 16, 0));
            path1.addWaypoint(new Pose2D(-16, 32, 0));
            path1.addWaypoint(new Pose2D(-16, 48, 0));

            path1.followPath(Trajectory.PATH_TYPE.PURE_PURSUIT, CornettCore.DIRECTION.FORWARD, 5, 1);

            stop();
        }
    }
}
