package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class Robot extends OpMode {

    private DcMotorEx backLeft, backRight, frontLeft, frontRight;

    private DcMotorEx leftEncoder, rightEncoder, lateralEncoder;

    private DcMotorEx liftMotor;
    private Servo rotationServo;
    private Servo grabberServo;

    private DcMotor intake;
    private Servo intakeServo;

    private DcMotorEx duck;

    private BNO055IMU imu;
    private Orientation angles;

    public ColorSensor freightSensor_color;
    public DistanceSensor freightSensor_distance;

    public HardwareMap hwMap;
    public SampleMecanumDrive driveTrain;
    public IMU IMU;
    public Lift lift;
    public Intake intakeSys;
    public DuckMotor spinMotor;
    public GamepadEx driveController;
    public GamepadEx operatorController;

    public static FtcDashboard dashboard;

    @Override
    public void init() {

    }

    @Override
    public void loop() throws InterruptedException {

    }

    public enum controlType{ROBOT, FIELD}

    public Robot(HardwareMap hardwareMap) {
        hwMap = hardwareMap;

        frontRight = hardwareMap.get(DcMotorEx.class, "fr");
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft = hardwareMap.get(DcMotorEx.class, "fl");
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backLeft = hardwareMap.get(DcMotorEx.class, "bl");
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backRight = hardwareMap.get(DcMotorEx.class, "br");
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //rotationServo = hardwareMap.servo.get("rotateServo");
        //grabberServo = hardwareMap.servo.get("grabber");

        intake = hardwareMap.dcMotor.get("intakeMotor");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intakeServo = hardwareMap.servo.get("intakeServo");

        //freightSensor_color = hardwareMap.get(ColorSensor.class, "color");
        //freightSensor_distance = hardwareMap.get(DistanceSensor.class, "color");

        duck = hardwareMap.get(DcMotorEx.class, "duckMotor");
        duck.setDirection(DcMotorSimple.Direction.FORWARD);
        duck.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        duck.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lateralEncoder = frontLeft;
        //lateralEncoder.setDirection(DcMotorSimple.Direction.REVERSE);

        rightEncoder = backRight;
        //rightEncoder.setDirection(DcMotorSimple.Direction.REVERSE);

        leftEncoder = backLeft;
        //leftEncoder.setDirection(DcMotorSimple.Direction.REVERSE);

        driveTrain = new SampleMecanumDrive(hardwareMap);
        IMU = new IMU(imu);
        lift = new Lift(this);
        intakeSys = new Intake(this);
        spinMotor = new DuckMotor(this);
        driveController = new GamepadEx(gamepad1);
        operatorController = new GamepadEx(gamepad2);
    }

    public void update() {

    }

    public void setSTART_POSITION(Pose2d START) {
        driveTrain.setPoseEstimate(START);
    }

    private void resetDriveEncoders() {
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lateralEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lateralEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void stopDrive() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public DcMotor getIntake() {
        return intake;
    }

    public Servo getIntakeServo() {
        return intakeServo;
    }

    public DcMotorEx getBackLeft() {
        return backLeft;
    }

    public DcMotorEx getBackRight() {
        return backRight;
    }

    public DcMotorEx getFrontLeft() {
        return frontLeft;
    }

    public DcMotorEx getFrontRight() {
        return frontRight;
    }

    public DcMotorEx getLeftEncoder() {
        return leftEncoder;
    }

    public DcMotorEx getRightEncoder() {
        return rightEncoder;
    }

    public DcMotorEx getLateralEncoder() {
        return leftEncoder;
    }

    public DcMotorEx getLiftMotor() {
        return liftMotor;
    }

    public DcMotorEx getDuck() {
        return duck;
    }

    public double getRight() {
        return rightEncoder.getCurrentPosition(); // invert if necessary
    }

    public double getLeft() {
        return leftEncoder.getCurrentPosition();  // invert if necessary
    }

    public double getLateral() {
        return lateralEncoder.getCurrentPosition();  // invert if necessary
    }

}