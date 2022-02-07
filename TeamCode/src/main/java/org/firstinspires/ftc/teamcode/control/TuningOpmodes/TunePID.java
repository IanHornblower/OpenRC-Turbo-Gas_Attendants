package org.firstinspires.ftc.teamcode.control.TuningOpmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.control.CornettCore;
import org.firstinspires.ftc.teamcode.control.Trajectory;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.AngleUtil;
import org.firstinspires.ftc.teamcode.util.Controller;
import static org.firstinspires.ftc.teamcode.util.Controller.*;
import static org.firstinspires.ftc.teamcode.util.MathUtil.roundPlaces;

@Disabled
@Config
@TeleOp(name = "TunePID", group = "Tuning")
public class TunePID extends LinearOpMode {

    public enum type{ROTATIONAL, DIRECTIONAL, BOTH, DIFFY}

    type PIDSelector;


    public static double scaleBox = 2;
    public static double waitPeriod = 200;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        robot.setSTART_POSITION(new Pose2D(0, 0, AngleUtil.interpretAngle(90)));

        Trajectory joe = new Trajectory(robot, robot.START_POSITION);

        joe.addWaypoint(new Point(0, 16));
        joe.addWaypoint(new Point(12, 24));
        joe.addWaypoint(new Point(12, 45));

        joe.retrace();

        boolean ran = false;
        telemetry.addLine("Press X for tuning heading PID\nPress B for Tuning xy PID\nPress Y for both\nPress A for Tuning Diffy");
        telemetry.update();

        do {
            if (gamepad1.x || gamepad2.x) {
                ran = true;
                PIDSelector = type.ROTATIONAL;
            }

            if (gamepad1.b || gamepad2.b) {
                ran = true;
                PIDSelector = type.DIRECTIONAL;
            }

            if (gamepad1.y || gamepad2.y || gamepad1.a || gamepad2.a) {
                ran = true;
                PIDSelector = type.BOTH;
            }

        } while(!ran);

        telemetry.clear();

        telemetry.addData("Tuning", PIDSelector);
        telemetry.update();

        CornettCore motionProfile = new CornettCore(robot);

        robot.intakeSys.regularFreightIntake();
        robot.lift.startServo();

        waitForStart();

        while(opModeIsActive()) {
            switch (PIDSelector) {
                case ROTATIONAL:
                    robot.updateOdometry();
                    robot.PIDEx.update();

                    motionProfile.rotateSync(Math.toRadians(0), Math.toRadians(0.5));

                    sleep((long) waitPeriod);

                    motionProfile.rotateSync(Math.toRadians(90), Math.toRadians(0.5));
                    sleep((long) waitPeriod);

                    break;
                case DIRECTIONAL:
                    robot.updateOdometry();
                    robot.PIDEx.update();

                    motionProfile.runToPositionSync(10*scaleBox, 0, Math.toRadians(90), 0.5);
                    sleep((long) waitPeriod);
                    motionProfile.runToPositionSync(10*scaleBox, 10*scaleBox, Math.toRadians(90), 0.5);
                    sleep((long) waitPeriod);
                    motionProfile.runToPositionSync(0, 10*scaleBox, Math.toRadians(90), 0.5);
                    sleep((long) waitPeriod);
                    motionProfile.runToPositionSync(0, 0, Math.toRadians(90), 0.5);
                    sleep((long) waitPeriod);
                    break;
                case DIFFY:
                    robot.updateOdometry();
                    robot.PIDEx.update();

                    joe.retrace().followPath(Trajectory.PATH_TYPE.PURE_PURSUIT, CornettCore.DIRECTION.FORWARD, 8, 1);

                    sleep((long) waitPeriod);

                    joe.retrace().followPath(Trajectory.PATH_TYPE.PURE_PURSUIT, CornettCore.DIRECTION.BACKWARD, 8, 1);

                    sleep((long) waitPeriod);
                    break;
                case BOTH:
                    robot.updateOdometry();
                    robot.PIDEx.update();

                    motionProfile.runToPositionSync(10*scaleBox, 0, Math.toRadians(0), 0.5);
                    sleep((long) waitPeriod);
                    motionProfile.runToPositionSync(10*scaleBox, 10*scaleBox, Math.toRadians(90), 0.5);
                    sleep((long) waitPeriod);
                    motionProfile.runToPositionSync(0, 10*scaleBox, Math.toRadians(180), 0.5);
                    sleep((long) waitPeriod);
                    motionProfile.runToPositionSync(0, 0, Math.toRadians(270), 0.5);
                    sleep((long) waitPeriod);
                    break;

                default:
                    telemetry.addLine("Well something is fucked?");
                    telemetry.addLine("Prolly ur enums dumbfuck");
                    telemetry.update();

                    PIDSelector = type.ROTATIONAL;
            }
        }
    }
}
