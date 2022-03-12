package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.util.Init.init;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.atomic.AtomicReference;

@Config
public class Lift {

    DcMotor lift;
    Servo boxServo;
    Robot robot;

    public static double power = 0.4;
    public static double min = 0.2;
    public static double max = 0.6;
    public static double threshold = 20;

    // Dont use
    public static double liftStart = 5;
    public static double liftOne = 300/0.71724137931;
    public static double liftTwo = 600/0.71724137931;
    // Use
    public static double liftThree = 1420;
    public static double liftPrimed = liftThree;

    public static double servoStart = 0.81;
    public static double servoPrimed = 0.63;
    public static double servoDropped = 0.31;
    public static double servoDriving = 0.69;
    public static double servoSuperDrop = 0.2;

    public static double kP = 0.0045;
    public static double kI = 0.085;
    public static double kD = 0.0003;
    public static double kF = 0.0;

    AtomicReference<Double> staticError = new AtomicReference<>((double) 0);

    public enum LIFT {
        START,
        PRIMED,
        D1,
        D2,
        D3
    }

    public enum SERVO {
        START,
        PRIMED,
        DROPPED
    }

    LIFT liftState;
    SERVO servoState;

    PIDFController pid = new PIDFController(kP, kI, kD,kF);

    public Lift(Robot robot) {
        this.lift = robot.getLiftMotor();
        this.boxServo = robot.getDropServo();
        this.robot = robot;
        liftState = LIFT.START;
        servoState = SERVO.START;
    }

    public void drop() {
        boxServo.setPosition(servoDropped);
    }

    public void primeServo() {
        boxServo.setPosition(servoPrimed);
    }

    public void drivingServo() {
        boxServo.setPosition(servoDriving);
    }

    public void startServo() {
        boxServo.setPosition(servoStart);
    }

    public void superDrop() {
        boxServo.setPosition(servoSuperDrop);
    }

    public void setPosition(double position) throws InterruptedException {
        setRawPosition(position);
    }

    public void setStinkyRawPosition(double position, double min, double max, double tolerance) throws InterruptedException {
        // TODO: RAMP UP

        init(()-> {
            staticError.set(position - lift.getCurrentPosition());
        });

        double error = position - lift.getCurrentPosition();

        double rampDownPosition = staticError.get() * 0.5;

        if(error < rampDownPosition) {
            power = min;
        }
        else {
            power = max;
        }

        if(position - lift.getCurrentPosition() > 0 && Math.abs(lift.getCurrentPosition() - position) > threshold) {
            //lift.setPower(-power);
        }
        else if(position - lift.getCurrentPosition() < 0 && Math.abs(lift.getCurrentPosition() - position) > threshold) {
            //lift.setPower(power);
        }
        else {
            //lift.setPower(0.0);
        }
    }

    public void setRawPosition(double position) {
        //pid.setPIDF(kP, kI, kD,kF); // For testing
        lift.setPower(-pid.calculate(lift.getCurrentPosition(), position));

    }

    public void SyncSetPosition(double position) throws InterruptedException {
        while(Math.abs(lift.getCurrentPosition() - position) > threshold) {
            setPosition(position);
        }
        lift.setPower(0.0);
    }

    public boolean isDown() {
        return lift.getCurrentPosition() < 150;
    }

    public void prime() throws InterruptedException {
        if (liftState == LIFT.START) {
            setPosition(liftPrimed);
            if (Math.abs(lift.getCurrentPosition() - liftPrimed) < threshold) {
                boxServo.setPosition(servoPrimed);
                liftState = LIFT.PRIMED;
            }
        } else {
            setPosition(liftPrimed);
            boxServo.setPosition(servoPrimed);
            if(Math.abs(lift.getCurrentPosition() - liftPrimed) < threshold) liftState = LIFT.PRIMED;
        }
    }
    public void retract() throws InterruptedException {
        switch (liftState) {
            case START:
                break;
            case PRIMED:
                boxServo.setPosition(servoStart);
                setPosition(liftStart);
            default:
                boxServo.setPosition(servoStart);
                setPosition(liftStart);
        }
    }

    public void prime(LIFT state) {
        Thread t1 = new Thread(()-> {
            switch(state) {
                case D1:
                    try {
                        setOne();
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    boxServo.setPosition(servoPrimed);
                    break;

                case D2:
                    try {
                        setTwo();
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    boxServo.setPosition(servoPrimed);
                    break;

                case D3:
                    try {
                        setThree();
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    boxServo.setPosition(servoPrimed);
                    break;
            }
        });
    }

    public void setOne() throws InterruptedException {
        //while(Math.abs(lift.getCurrentPosition() - liftOne) > threshold) {
        setPosition(liftOne);
        //}
    }

    public void setTwo() throws InterruptedException {
        //while(Math.abs(lift.getCurrentPosition() - liftTwo) > threshold) {
        setPosition(liftTwo);
        //}
    }

    public void setThree() throws InterruptedException {
        //while(Math.abs(lift.getCurrentPosition() - liftThree) > threshold) {
        setPosition(liftThree);
        //}
    }

    public void setStart() throws InterruptedException {
        //while(Math.abs(lift.getCurrentPosition() - liftStart) > threshold) {
        setPosition(liftStart);
        //}
    }
}
