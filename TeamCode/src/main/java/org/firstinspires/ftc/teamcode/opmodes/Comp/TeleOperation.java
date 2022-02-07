package org.firstinspires.ftc.teamcode.opmodes.Comp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.system.RefCounted;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.control.Coefficients;
import org.firstinspires.ftc.teamcode.dashboard.Field;
import org.firstinspires.ftc.teamcode.hardware.Blinkin;
import org.firstinspires.ftc.teamcode.hardware.FreightDetector;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.intake;
import org.firstinspires.ftc.teamcode.hardware.lift;
import org.firstinspires.ftc.teamcode.math.Curve;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.AngleUtil;
import org.firstinspires.ftc.teamcode.util.Controller;
import static org.firstinspires.ftc.teamcode.util.Controller.*;
import static org.firstinspires.ftc.teamcode.util.MathUtil.roundPlaces;
import static org.firstinspires.ftc.teamcode.hardware.lift.*;
import static org.firstinspires.ftc.teamcode.hardware.lift.*;

import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.MiniPID;
import org.firstinspires.ftc.teamcode.util.Timer;

@Config
@TeleOp(name = "TeleOp", group = "Comp")
public class TeleOperation extends LinearOpMode {

    public static double movementSens = 1;
    public static double turnSens = 0.5;

    public static double slowed_movementSens = 0.41;
    public static double slowed_turnSens = 0.29;

    public static double position = liftStart;
    public static boolean dashboard = true;
    boolean down = true;
    boolean isMoving = false;
    boolean intakeDown = false;

    private enum DRIVE {
        FIELD,
        ROBOT
    }

    private enum Speed {
        SLOW,
        REGULAR,
        FAST
    }

    Speed speed = Speed.REGULAR;
    double turn = 0;

    public static enum LIFTSTATE {
        START,
        ONE,
        TWO,
        THREE;
    }

    public static enum TELEOPSTATE {
        START,
        PRIME,

    }

    LIFTSTATE liftPos = LIFTSTATE.START;

    DRIVE driveState = DRIVE.ROBOT;

    final LIFTSTATE[] lift = {LIFTSTATE.ONE};

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Timer runTime = new Timer();

        robot.setSTART_POSITION(PoseStorage.autoEnd);
        double modTheta = robot.START_POSITION.heading;

        switch (MatchConfig.side) {
            case RED:
                robot.getDuck().setDirection(DcMotorSimple.Direction.REVERSE);
                modTheta = -Math.PI/2+modTheta;
                break;
            case BLUE:
                robot.getDuck().setDirection(DcMotorSimple.Direction.FORWARD);
                modTheta = Math.PI/2+modTheta;
        }

        robot.freightDetector.run();

        waitForStart();

        robot.lift.startServo();

        runTime.start();

        //runTime.addSeconds(0);

        while(opModeIsActive()) {
            Thread t1 = new Thread(() -> {
                switch (lift[0]) {
                    case ONE:
                        sleep(200);
                        if(gamepad2.dpad_right) lift[0] = LIFTSTATE.TWO;
                        if(gamepad2.dpad_left) lift[0] = LIFTSTATE.THREE;
                        break;
                    case TWO:
                        sleep(200);
                        if(gamepad2.dpad_right) lift[0] = LIFTSTATE.THREE;
                        if(gamepad2.dpad_left) lift[0]  = LIFTSTATE.ONE;
                        break;
                    case THREE:
                        sleep(200);
                        if(gamepad2.dpad_right)  lift[0] = LIFTSTATE.ONE;
                        if(gamepad2.dpad_left) lift[0] = LIFTSTATE.TWO;
                        break;
                }
            });

            t1.start();

            robot.updateOdometry();

            // Drive Train

            double leftX = AngleUtil.powRetainingSign(Controller.deadZone(gamepad1.left_stick_x, 0.1), LEFT_TRIGGER_X_POW);
            double leftY = AngleUtil.powRetainingSign(Controller.deadZone(-gamepad1.left_stick_y, 0.1), LEFT_TRIGGER_Y_POW);
            turn = Controller.deadZone(gamepad1.right_stick_x, 0.1);

            if(driveState == DRIVE.FIELD) {
                robot.DriveTrain.driveFieldCentric(leftX, leftY, turn, modTheta);
                if(gamepad1.square) driveState = DRIVE.ROBOT;
            }
            else {
                robot.DriveTrain.setMotorPowers(leftX*movementSens, leftY*movementSens, turn*turnSens);
                if(gamepad1.triangle) driveState = DRIVE.FIELD;
            }

            isMoving = leftX > 0.1 || leftY > 0.1;

            if(gamepad1.cross) {
                movementSens = 0.78;
                turnSens = 0.59;
                speed = Speed.REGULAR;
            }

            if(gamepad1.circle) {
                movementSens = slowed_movementSens;
                turnSens = slowed_turnSens;
                speed = Speed.SLOW;
            }

            // Duck Motor
            robot.spinMotor.run(gamepad2.left_bumper, gamepad2.right_bumper);

            // Intake

            if(gamepad2.dpad_down)  {
                intakeDown = true;
                robot.intakeSys.regularFreightIntake();
            }

            if(gamepad2.dpad_up) {
                intakeDown = false;
                robot.intakeSys.raiseIntake();
            }

            if(gamepad1.dpad_left || gamepad1.dpad_right) robot.intakeSys.inAirIntake();

            robot.intakeSys.run(gamepad2.left_trigger, gamepad2.right_trigger, down);

            // Lift

            robot.lift.setPosition(position);

            if(gamepad2.square && !isMoving && intakeDown) {
                down = false;
                if (lift[0] == LIFTSTATE.ONE) {
                    position = liftOne;
                    robot.lift.primeServo();
                } else if (lift[0] == LIFTSTATE.TWO) {
                    position = liftTwo;
                    robot.lift.primeServo();
                } else if (lift[0] == LIFTSTATE.THREE) {
                    position = liftThree;
                    robot.lift.primeServo();
                }
            }

            if(gamepad2.circle && !down && !isMoving) {
                robot.lift.drop();
            }

            if(gamepad2.triangle && !isMoving && intakeDown) {
                down = false;
                position = liftThree;
                robot.lift.primeServo();
            }

            if(gamepad2.cross && intakeDown) {
                down = true;
                robot.lift.startServo();
                position = liftStart;
            }

            // Timer Settings

            if (runTime.currentSeconds() > 75 && runTime.currentSeconds() < 90) {  /// 75 -> 90
                FreightDetector.isRunning = false;
                gamepad1.rumble(8*1000);
                gamepad2.rumble(8*1000);
                Blinkin.Driver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_LARSON_SCANNER);
            }
            else if (runTime.currentSeconds() > 90 && runTime.currentSeconds() < 110) {  // 90 -> 110
                FreightDetector.isRunning = false;
                Blinkin.Driver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_TWINKLES);

            }
            else if(runTime.currentSeconds() > 110 && runTime.currentSeconds() < 120) {
                FreightDetector.isRunning = false;
                Blinkin.Driver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                Blinkin.Driver.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE);
            }
            else {
                FreightDetector.isRunning = true;
            }




                //telemetry.addData("End Auto Pos", PoseStorage.autoEnd.toString());
            //telemetry.addData("Current Pos", robot.pos.toString());
            //telemetry.addData("IMU ANGLE", robot.IMU.getIMUHeading() + "DEGREES: " + Math.toDegrees(robot.IMU.getIMUHeading()));
            //telemetry.addData("Robot ANGLE", robot.pos.heading + "DEGREES: " + Math.toDegrees(robot.pos.heading));

            telemetry.addData("lift", robot.getLift().getCurrentPosition());
            telemetry.addData("Lift Level", lift[0].toString());
            telemetry.addData("Drive State", driveState.toString());
            //telemetry.addData("Drive Speed", speed.toString());
            telemetry.addData("Side", MatchConfig.side.toString());
            telemetry.addData("Run Time", (int)runTime.currentSeconds());
            telemetry.update();
        }
    }
}
