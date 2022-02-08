package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.util.Init.init;
import static org.firstinspires.ftc.teamcode.util.Time.timeout;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.MiniPID;
import org.firstinspires.ftc.teamcode.vision.FreightFrenzyCamera;

import java.util.concurrent.atomic.AtomicReference;

@Config
public class lift {

    DcMotor lift;
    Servo boxServo;
    MiniPID liftPID;
    Robot robot;

    public static double power = 0.4;
    public static double min = 0.2;
    public static double max = 0.6;
    public static double threshold = 20;

    public static double liftStart = 20;
    public static double liftOne = 300;
    public static double liftTwo = 600;
    public static double liftThree = 1020;
    public static double liftPrimed = liftThree;

    public static double servoStart = 0.81;
    public static double servoPrimed = 0.72;
    public static double servoDropped = 0.37;
    public static double servoSemiDrop = 0.43;

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

    public lift(Robot robot) {
        this.lift = robot.getLift();
        this.boxServo = robot.getBoxServo();
        this.robot = robot;
        //liftPID = new MiniPID(P, I, D);
        liftState = LIFT.START;
        servoState = SERVO.START;
    }

    public void setLiftPID(MiniPID pid) {
        this.liftPID = pid;
    }

    public void drop() {
        boxServo.setPosition(servoDropped);
    }

    public void primeServo() {
        boxServo.setPosition(servoPrimed);
    }

    public void lowerServo() {
        boxServo.setPosition(servoSemiDrop);
    }

    public void startServo() {
        boxServo.setPosition(servoStart);
    }
    
    public void setPosition(double position) throws InterruptedException {
        setRawPosition(position, min, max, threshold);
    }

    public void setRawPosition(double position, double min, double max, double tolerance) throws InterruptedException {
        // TODO: RAMP UP

        init(()-> {
            staticError.set(position - robot.getLift().getCurrentPosition());
        });

        double error = position - robot.getLift().getCurrentPosition();

        double rampDownPosition = staticError.get() * 0.5;

        if(error < rampDownPosition) {
            power = min;
        }
        else {
            power = max;
        }

        if(position - robot.getLift().getCurrentPosition() > 0 && Math.abs(robot.getLift().getCurrentPosition() - position) > threshold) {
            robot.getLift().setPower(-power);
        }
        else if(position - robot.getLift().getCurrentPosition() < 0 && Math.abs(robot.getLift().getCurrentPosition() - position) > threshold) {
            robot.getLift().setPower(power);
        }
        else {
            robot.getLift().setPower(0.0);
        }


    }

    public void SyncSetPosition(double position) throws InterruptedException {
        while(Math.abs(robot.getLift().getCurrentPosition() - position) > threshold) {
            setPosition(position);
        }
        robot.getLift().setPower(0.0);
    }

    public boolean isDown() {
        return robot.getLift().getCurrentPosition() < 150;
    }

    public void prime() throws InterruptedException {
        if (liftState == LIFT.START) {
            setPosition(liftPrimed);
            if (Math.abs(robot.getLift().getCurrentPosition() - liftPrimed) < threshold) {
                boxServo.setPosition(servoPrimed);
                liftState = LIFT.PRIMED;
            }
        } else {
            setPosition(liftPrimed);
            boxServo.setPosition(servoPrimed);
            if(Math.abs(robot.getLift().getCurrentPosition() - liftPrimed) < threshold) liftState = LIFT.PRIMED;
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

    public void doAuto(FreightFrenzyCamera.position location) throws InterruptedException {
        switch (location) {
            case A:
                setOne();
                boxServo.setPosition(servoDropped);
                setStart();
                break;
            case B:
                setTwo();
                boxServo.setPosition(servoDropped);
                setStart();
                break;
            case C:
                setThree();
                boxServo.setPosition(servoDropped);
                setStart();
                break;
        }
    }
}