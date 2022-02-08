package org.firstinspires.ftc.teamcode.opmodes.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.dashboard.Field;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.lift;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.AngleUtil;
import org.firstinspires.ftc.teamcode.util.Controller;
import org.firstinspires.ftc.teamcode.util.MiniPID;

import static org.firstinspires.ftc.teamcode.util.Controller.*;
import static org.firstinspires.ftc.teamcode.util.Time.await;

@Disabled
@Config
@TeleOp(name = "Testing OpMode", group = "Testing")
public class TestTeleOp extends LinearOpMode {
    public static double position = 0;

    public static double p = 0.007, i = 0, d = 5;

    public void lowerDrop() {
        Thread t1 = new Thread(()-> {
            Robot robot = new Robot(hardwareMap);
            robot.lift.drop();
            try {
                await(200, ()-> {
                    robot.lift.lowerServo();
                });
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        });

    }

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        robot.setSTART_POSITION(new Pose2D(0, 0, AngleUtil.interpretAngle(90)));

        robot.getLift().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getLift().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.lift.setLiftPID(new MiniPID(p, i, d));

        waitForStart();

        while(opModeIsActive()) {

            robot.freightDetector.run();

            TelemetryPacket packet = new TelemetryPacket();

            FtcDashboard dashboard = FtcDashboard.getInstance();
            telemetry = dashboard.getTelemetry();

            Field field = new Field(packet);

            robot.updateOdometry();
            robot.updateAccumulatedHeading();
            robot.PIDEx.update();

            double leftX = AngleUtil.powRetainingSign(Controller.deadZone(gamepad1.left_stick_x, 0.1), LEFT_TRIGGER_X_POW);
            double leftY = AngleUtil.powRetainingSign(Controller.deadZone(-gamepad1.left_stick_y, 0.1), LEFT_TRIGGER_Y_POW);
            double turn = Controller.deadZone(gamepad1.right_stick_x, 0.1);

            /*

            if(gamepad1.square) {
                robot.getFrontLeft().setPower(1);
            }

            else if(gamepad1.triangle) {
                robot.getFrontRight().setPower(1);
            }

            else if(gamepad1.circle) {
                robot.getBackRight().setPower(1);
            }

            else if(gamepad1.cross) {
                robot.getBackLeft().setPower(1);
            }

            else {
                robot.stopDrive();
            }

             */

           robot.lift.setPosition(position);

           telemetry.addData("State", robot.getLift().getCurrentPosition());
           telemetry.addData("Refrence", position);
           telemetry.addData("Error", position - robot.getLift().getCurrentPosition());

           telemetry.addData("Distance", robot.freightDetector.distance());


            if(gamepad1.square) {
                robot.lift.drop();
            }

            if(gamepad1.triangle) {
                robot.lift.primeServo();
            }
            if(gamepad1.circle) {
                robot.lift.startServo();
            }

            if(gamepad1.cross) {
                robot.freightDetector.close();
            }


            //telemetry.addData("\nXYH", robot.pos.toString());
            telemetry.addData("Left", robot.getLeftEncoder().getCurrentPosition());
            telemetry.addData("Right", robot.getRightEncoder().getCurrentPosition());
            telemetry.addData("Back", robot.getFrontEncoder().getCurrentPosition());
            telemetry.update();
        }
    }
}
