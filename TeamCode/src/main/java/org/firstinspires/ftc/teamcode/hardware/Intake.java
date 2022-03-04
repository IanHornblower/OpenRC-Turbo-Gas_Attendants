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
public class Intake {

    Robot robot;
    DcMotor intake;
    Servo intakeServo;

    public static double UP = 0.0;
    public static double DOWN = 0.4;
    public static double SPEED = 1.0;

    private IntakeState state;

    public enum IntakeState {
        UP,
        DOWN,
        IN,
        OUT,
        OFF
    }
    public intake(Robot robot) {
        this.robot = robot;
        this.intake = robot.getIntake();
        this.intakeServo = robot.getIntakeServo();
        state = IntakeState.DOWN;
    }


    public void update() {
        switch (state) {
            case UP:
                robot.getIntakeServo().setPosition(UP);
                break;
            case DOWN:
                robot.getIntakeServo().setPosition(DOWN);
                break;
            case IN:
                setIntakePower(SPEED);
                break;
            case OFF:
                setIntakePower(0.0);
                break;
            case OUT:
                setIntakePower(-SPEED);
                break;
        }

    }

    public void setState(IntakeState state) {
        this.state = state;
    }

    public IntakeState state(){
        return state;
    }

    public boolean isDown() {
        return robot.getIntakeServo().getPosition() != 0.0;
    }

    public void setIntakePower(double power) {
        robot.getIntake().setPower(power);
    }

    /*
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
     */
    public void raiseIntake() {
        intakeServo.setPosition(UP);
    }

    public void regularFreightIntake() {
        robot.getIntakeServo().setPosition(DOWN);
    }
}

    // Feedback - Color Sensor
