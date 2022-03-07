package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.gamepad.Joystick;
import org.firstinspires.ftc.teamcode.hardware.Robot;

@Config
@TeleOp
public class TeleOperation extends LinearOpMode {

    public static double liftPosition = 0;
    public static double duckVelocity = 0;

    public static double kP = 0.0018;
    public static double kF = 1;

    @Override
    public void runOpMode() throws  InterruptedException {
        Robot robot = new Robot(hardwareMap);
        robot.driveController.setGamepad(gamepad1);
        robot.operatorController.setGamepad(gamepad2);

        while (!opModeIsActive() && !isStopRequested()) {

        }

        waitForStart();

        ElapsedTime timer = new ElapsedTime();
        duckVelocity = 250;
        while (opModeIsActive() && !isStopRequested()) {

            /*
            Testing
             */

            telemetry = FtcDashboard.getInstance().getTelemetry();

            robot.lift.setPosition(liftPosition);


            telemetry.addData("State", robot.lift.getState());
            telemetry.addData("Arm Refrance", liftPosition);
            telemetry.addData("Arm State", robot.lift.motor().getCurrentPosition());
            telemetry.addData("Arm Error", robot.lift.error);
            double angle = robot.lift.tickToRadians(liftPosition) + Math.toRadians(44.3);
            telemetry.addData("Arm Refrance Angle", angle);
            telemetry.addData("Arm Angle State", robot.lift.tickToRadians(robot.lift.motor().getCurrentPosition())+ Math.toRadians(44.3));
            telemetry.update();

            // Driver Controller

            double leftX = -robot.driveController.leftJoystick.x();
            double leftY = -robot.driveController.leftJoystick.y();
            double turn =  -robot.driveController.rightJoystick.x()*0.8;

            robot.driveTrain.setWeightedDrivePower(new Pose2d(
                    leftY,
                    leftX,
                    turn
            ));

            // Operator Controller

            if(robot.operatorController.rightTrigger.isPressed()) {
                // Intake in
            }
            else if(robot.operatorController.leftTrigger.isPressed()) {
                // Intake out
            }
            else  {
                // Intake off
            }

            switch (robot.operatorController.dpad()) {
                case down:
                    // Return lift to intaking position
                    break;
                case left:
                    // Return lift to 1 position
                    break;
                case right:
                    // Return lift to 3 position
                    break;
                case up:
                    // Return lift to 2 position
                    break;
            }

            if(robot.operatorController.square()) {
                // Spin duck Right
            }
            if(robot.operatorController.triangle()) {
                // Spin duck left
            }

            if(robot.operatorController.gamepad().a) {
                robot.spinMotor.doDuckSpin();
            }

            if(robot.operatorController.leftJoystick.isPressed()) {
                // Intake up
            }

            if(robot.operatorController.rightJoystick.isPressed()) {
                // Intake down
            }

            // Set up manual control later

            // Timer based stuff

            if (timer.seconds() > 15 && timer.seconds() < 20) { // Change values
                // Blinkins
            }


        }
    }
}
