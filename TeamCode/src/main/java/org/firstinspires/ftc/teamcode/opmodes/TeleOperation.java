package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.gamepad.Joystick;
import org.firstinspires.ftc.teamcode.hardware.Blinkin;
import org.firstinspires.ftc.teamcode.hardware.FreightDetector;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Timer;

@Config
@TeleOp
public class TeleOperation extends LinearOpMode {

    public static double liftPosition = 0;
    public static double duckVelocity = 220;
    public static double capperPosition = 0;

    public static double grabber_open = 0.45;
    public static double grabber_close = 0;

    boolean isManual = false;
    boolean isIntakeRunning = false;
    boolean liftRaised = false;
    boolean isBucketInStart = true;
    boolean isDropping = false;

    @Override
    public void runOpMode() throws  InterruptedException {
        Robot robot = new Robot(hardwareMap);
        robot.driveController.setGamepad(gamepad1);
        robot.operatorController.setGamepad(gamepad2);

        robot.operatorController.leftJoystick.invert(Joystick.AXIS.y);
        robot.freightDetector.run();

        int sideMult = 1;

        while (!opModeIsActive() && !isStopRequested()) {
            if(MatchConfig.side == MatchConfig.SIDE.RED) {
                sideMult = 1;
            }
            else sideMult = -1;
        }

        Timer runTime = new Timer();
        waitForStart();

        runTime.start();
        while (opModeIsActive() && !isStopRequested()) {

            /*
            Testing
             */

            telemetry = FtcDashboard.getInstance().getTelemetry();

            robot.lift.setPosition(liftPosition);

            telemetry.addData("lift Refrance", liftPosition);
            telemetry.addData("lift state", robot.getLiftMotor().getCurrentPosition());

            telemetry.addData("Capper Refrance", capperPosition);
            telemetry.addData("Capper state", robot.getLiftMotor().getCurrentPosition());
            telemetry.addData("Zero", 0);
            telemetry.update();

            /*
                Handle States
             */

            if(robot.getIntake().getPower() > 0.1) {
                isIntakeRunning = true;
            }
            else {
                isIntakeRunning = false;
            }

            if(robot.getLiftMotor().getCurrentPosition() > 700) {
                liftRaised = true;
            }
            else {
                liftRaised = false;
            }

            if(robot.getDropServo().getPosition() == 0.81) {
                isBucketInStart = true;
            }
            else {
                isBucketInStart = false;
            }

            if(robot.getDropServo().getPosition() == 0.31) {
                isDropping = true;
            }
            else {
                isDropping = false;
            }




            /*
                Controller Stuff
             */

            // Driver Controller

            double leftX = -robot.driveController.leftJoystick.x();
            double leftY = -robot.driveController.leftJoystick.y();
            double turn =  -robot.driveController.rightJoystick.x()*0.8;

            if(robot.driveController.leftTrigger.isPressed()) {
                leftX += 0.5*sideMult;
            }

            if(robot.driveController.rightTrigger.isPressed()) {
                leftX-= 0.5*sideMult;
            }

            robot.driveTrain.setWeightedDrivePower(new Pose2d(
                    leftY,
                    leftX,
                    turn
            ));

            // Operator Controller

            if(robot.operatorController.rightTrigger.isPressed()) {
                robot.intakeSys.setIntakePower(1.0);
            }
            else if(robot.operatorController.leftTrigger.isPressed()) {
                robot.intakeSys.setIntakePower(-1.0);
            }
            else  {
                robot.intakeSys.setIntakePower(0.0);
            }

            if(!isIntakeRunning && !liftRaised) {
                robot.lift.drivingServo();
            }
            else if(isIntakeRunning) {
                robot.lift.startServo();
            }

            double rawDrab = robot.operatorController.leftJoystick.y();
            double drabPower = ((Math.abs(rawDrab)/rawDrab) * Math.sqrt(rawDrab))/2;

            robot.getGrabMotor().setPower(drabPower);

            /*

            if(liftRaised) {
                robot.lift.primeServo();
            }
            else if(!liftRaised && !isIntakeRunning) {
                robot.lift.startServo();
            }

             */

            if(!isManual && !isDropping) {
                switch (robot.operatorController.dpad()) {
                    case down:
                        liftPosition = 10;
                        break;
                    case left:

                        break;
                    case right:

                        break;
                    case up:
                        liftPosition = Lift.liftThree;
                        break;
                }

                if(robot.driveController.cross()) {
                    isManual = !isManual;
                }
            }
            else {
                switch (robot.operatorController.dpad()) {
                    case down:
                        robot.getLiftMotor().setPower(-0.3);
                        break;
                    case left:
                        // Return lift to 1 position
                        break;
                    case right:
                        // Return lift to 3 position
                        break;
                    case up:
                        robot.getLiftMotor().setPower(0.3);
                        break;
                    case unpressed:
                        robot.getLiftMotor().setPower(0.0);
                }

                if(robot.driveController.cross()) {
                    isManual = !isManual;
                }
            }

            if(robot.driveController.triangle()) {
                robot.getLiftMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.getLiftMotor().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if(robot.driveController.square()) {
                // close capper
                robot.capper.setGrabberPosition(grabber_close);
            }

            if(robot.driveController.circle()) {
                // open capper
                robot.capper.setGrabberPosition(grabber_open);
            }

            //robot.operatorController.leftJoystick.invert(Joystick.AXIS.x);

            if(robot.operatorController.square()) {
                robot.lift.startServo();
            }

            if(robot.operatorController.triangle()) {
                robot.lift.drivingServo();
            }

            if(robot.operatorController.circle()) {
                robot.lift.drop();
            }

            if(robot.operatorController.left_bumper()) {
                robot.spinMotor.setVelocity(-duckVelocity*sideMult);
            }
            else if(robot.operatorController.right_bumper()) {
                robot.spinMotor.setVelocity(duckVelocity*sideMult);
            }
            else {
                robot.spinMotor.setVelocity(0.0);
            }

            if(robot.operatorController.cross()) {
                robot.lift.superDrop();
            }

            if(robot.operatorController.leftJoystick.isPressed()) {
                robot.intakeSys.raiseIntake();
            }

            if(robot.operatorController.rightJoystick.isPressed()) {
                robot.intakeSys.regularFreightIntake();
            }

            // Set up manual control later

            // Timer based stuff

            if (runTime.currentSeconds() > 75 && runTime.currentSeconds() < 90) {  /// 75 -> 90
                FreightDetector.isRunning = false;
                gamepad1.rumble(100);
                gamepad2.rumble(100);
                robot.blinkin.Driver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_LARSON_SCANNER);
            }
            else if (runTime.currentSeconds() > 90 && runTime.currentSeconds() < 110) {  // 90 -> 110
                robot.blinkin.Driver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_TWINKLES);

            }
            else if(runTime.currentSeconds() > 110 && runTime.currentSeconds() < 120) {
                robot.blinkin.Driver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                robot.blinkin.Driver.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE);
            }
            else {
                FreightDetector.isRunning = true;
            }

            telemetry.addData("Lift Automatic", isManual);
            telemetry.addData("Run Time", (int)runTime.currentSeconds());
            telemetry.addData("Has Freight", robot.freightDetector.hasFreight());
            //telemetry.update();

        }
    }
}
