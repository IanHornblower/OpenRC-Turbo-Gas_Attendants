package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.CallSuper;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class intake extends OpMode {

    Robot robot;
    DcMotor intake;
    Servo intakeServo;

    public static double ON = -1;

    public static double UP = 0.0;
    public static double RAISED = 0.2;
    public static double REGULAR_DOWN = 0.4;


    public intake(Robot robot) {
        this.robot = robot;
        this.intake = robot.getIntake();
        this.intakeServo = robot.getIntakeServo();
    }

    public void setIntakePower(double power) {
        robot.getIntake().setPower(power);
    }

    public void startIntake() {
        setIntakePower(ON);
    }

    public void reverseIntake() {
        setIntakePower(-ON);
    }

    public void stopIntake() {
        setIntakePower(0);
    }

    public void run(double left, double right, boolean down) {
        if(left > 0.1 && down) {
            intake.setPower(left*0.67);
        }
        else if (right > 0.1 && down) {
            intake.setPower(-right*0.67);
        }
        else {
            intake.setPower(0);
        }
    }

    public void raiseIntake() {
        intakeServo.setPosition(UP);
    }


    public void inAirIntake() {
        robot.getIntakeServo().setPosition(RAISED);
    }

    public void regularFreightIntake() {
        robot.getIntakeServo().setPosition(REGULAR_DOWN);
    }

    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}
