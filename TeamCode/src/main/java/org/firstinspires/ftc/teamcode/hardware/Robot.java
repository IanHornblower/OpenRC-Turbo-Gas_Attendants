package org.firstinspires.ftc.teamcode.hardware;

import android.os.health.ServiceHealthStats;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.control.Coefficients;
import org.firstinspires.ftc.teamcode.dashboard.Field;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.AngleUtil;
import org.firstinspires.ftc.teamcode.util.Time;

public class Robot extends OpMode {
    private DcMotorEx backLeft, backRight, frontLeft, frontRight;

    private DcMotorEx leftEncoder, rightEncoder, lateralEncoder;

    private DcMotor liftMotor;
    private Servo boxServo;

    private DcMotor intake;
    private Servo intakeServo;

    private DcMotor duck;

    private BNO055IMU imu;
    private Orientation angles;

    public Coefficients PIDEx;

    public FreightDetector freightDetector;
    public ColorSensor freightSensor_color;
    public DistanceSensor freightSensor_distance;

    private double previousHeading;

    public HardwareMap hwMap;
    public DriveTrain DriveTrain;
    public IMU IMU;
    public org.firstinspires.ftc.teamcode.hardware.lift lift;
    public intake intakeSys;
    public spinMotor spinMotor;
    public Controller driveController;
    public Controller operatorController;

    public FtcDashboard dashboard;

    public enum controlType{ROBOT, FIELD}

    // Robot Kinematics

    // Odmometric Constraints
    public final static double L = 10.89;  // separation between left and right Encoder.
    public final static double lateralOffset = -6.65;  // offset between origin of robot and lateral Encoder.
    public final static double R = 0.688975;  // Encoder wheel radius.
    public final static double encoderTicksPerRev = 8192;  // Ticks read per revolution of REV Encoder.
    public final static double inchPerTick = 2.0 * Math.PI * R / encoderTicksPerRev;  // Inches traveled per tick moved.

    public static boolean dashboardTelemetry = false;

    public Robot(HardwareMap hardwareMap) {
        hwMap = hardwareMap;

        frontRight = hardwareMap.get(DcMotorEx.class, "fr");
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft = hardwareMap.get(DcMotorEx.class, "fl");
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backLeft = hardwareMap.get(DcMotorEx.class, "bl");
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backRight = hardwareMap.get(DcMotorEx.class, "br");
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftMotor = hardwareMap.dcMotor.get("lift");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        boxServo = hardwareMap.servo.get("boxServo");

        intake = hardwareMap.dcMotor.get("intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intakeServo = hardwareMap.servo.get("intakeServo");

        PIDEx = new Coefficients();

        freightSensor_color = hardwareMap.get(ColorSensor.class, "color");
        freightSensor_distance = hardwareMap.get(DistanceSensor.class, "color");

        duck = hardwareMap.dcMotor.get("duck");
        duck.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        duck.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        lateralEncoder = frontLeft;
        //lateralEncoder.setDirection(DcMotorSimple.Direction.REVERSE);

        rightEncoder = backRight;
        //rightEncoder.setDirection(DcMotorSimple.Direction.REVERSE);

        leftEncoder = backLeft;
        //leftEncoder.setDirection(DcMotorSimple.Direction.REVERSE);

        stopDrive();
        resetDriveEncoders();

        Time.reset();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        DriveTrain = new DriveTrain(this);
        IMU = new IMU(imu);
        lift = new lift(this);
        intakeSys = new intake(this);
        spinMotor = new spinMotor(this);
        freightDetector = new FreightDetector(this);
        driveController = new Controller(gamepad1);
        operatorController = new Controller(gamepad2);

        dashboard = FtcDashboard.getInstance();

        if(dashboardTelemetry) {
            telemetry = dashboard.getTelemetry();
        }
    }

    public ColorSensor getFreightSensor_color() {
        return freightSensor_color;
    }

    public DistanceSensor getFreightSensor_distance() {
        return freightSensor_distance;
    }

    public HardwareMap getHardwareMap() {
        return hwMap;
    }

    public BNO055IMU getImu() {
        return imu;
    }

    public DcMotor getBackLeft() {
        return backLeft;
    }

    public DcMotor getFrontLeft() {
        return frontLeft;
    }

    public DcMotor getBackRight() {
        return backRight;
    }

    public DcMotor getFrontRight() {
        return frontRight;
    }

    public DcMotor getLeftEncoder() {
        return leftEncoder;
    }

    public DcMotor getRightEncoder() {
        return rightEncoder;
    }

    public DcMotor getFrontEncoder() {
        return lateralEncoder;
    }

    public DcMotor getLift() {
        return liftMotor;
    }

    public Servo getBoxServo() {
        return boxServo;
    }

    public Servo getIntakeServo() {
        return intakeServo;
    }

    public DcMotor getIntake() {
        return intake;
    }

    public DcMotor getDuck() {
        return duck;
    }

    private void resetDriveEncoders() {
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lateralEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lateralEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // update variables
    public int currentRightPosition = 0;
    public int currentLeftPosition = 0;
    public int currentLateralPosition = 0;

    private int oldRightPosition = 0;
    private int oldLeftPosition = 0;
    private int oldLateralPosition = 0;

    public double accumulatedHeading = 0;
    public double accumulatedDistance = 0;

    public double leftVelocity = 0;
    public double rightVelocity = 0;
    public double lateralVelocity = 0;

    public double dxTraveled = 0;
    public double dyTraveled = 0;

    public Pose2D START_POSITION = new Pose2D(0, 0, AngleUtil.interpretAngle(90));  // Default

    public void setSTART_POSITION(Pose2D START) {
        START_POSITION = START;
        pos = START;
    }

    public Pose2D pos = START_POSITION;

    int i = 0;

    public void pass() {
        System.out.print(accumulatedDistance);
    }


    public void updateAccumulatedHeading() {
        double currentHeading = Math.toDegrees(pos.getHeading());

        double dHeading = currentHeading - previousHeading;

        if(dHeading < -180) {
            dHeading += 360;
        }
        else if(dHeading >= 180) {
            dHeading -=360;
        }

        accumulatedHeading -= dHeading;
        previousHeading = currentHeading;
    }

    public void updateOdometry() {
        this.PIDEx.update();
        currentRightPosition = rightEncoder.getCurrentPosition(); // Invert in Necessary
        currentLeftPosition = leftEncoder.getCurrentPosition(); // Invert in Necessary
        currentLateralPosition = lateralEncoder.getCurrentPosition(); // Invert in Necessary

        int dnRight = currentRightPosition - oldRightPosition;
        int dnLeft = currentLeftPosition - oldLeftPosition;
        int dnLateral = currentLateralPosition - oldLateralPosition;

        double dtheta = (dnLeft - dnRight) / L;
        double dx = (dnLeft + dnRight) / 2.0;
        double dy = dnLateral - lateralOffset * dtheta;

        dtheta *= inchPerTick;
        dx *= inchPerTick;
        dy *= inchPerTick;

        //double theta = pos.heading + (dtheta / 2.0);  // Does same thing as pos.heading | Might remove
        double dxTraveled = dx * Math.cos(pos.heading) - dy * Math.sin(pos.heading);
        double dyTraveled = dx * Math.sin(pos.heading) + dy * Math.cos(pos.heading);

        accumulatedDistance += Math.hypot(dxTraveled, dyTraveled);

        pos.x -= dxTraveled;  // Inverted cuz it was negative? :)
        pos.y += dyTraveled;
        pos.heading += dtheta;

        /*
        TelemetryPacket packet = new TelemetryPacket();

        Field field = new Field(packet);
        field.createCircularRobot(this.pos);

        packet = field.getPacket();

        packet.addLine("XYH: " + pos.toString());

        dashboard.sendTelemetryPacket(packet);
        */

        oldRightPosition = currentRightPosition;
        oldLeftPosition = currentLeftPosition;
        oldLateralPosition = currentLateralPosition;
    }

    public void update() {
        updateOdometry();
        updateAccumulatedHeading();

        driveController.update();
        operatorController.update();
    }

    public void updateEncoderVelocity() {
        leftVelocity = leftEncoder.getVelocity()*inchPerTick;
        rightVelocity = rightEncoder.getVelocity()*inchPerTick;
        lateralVelocity = lateralEncoder.getVelocity()*inchPerTick;
    }

    public void stopDrive() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}