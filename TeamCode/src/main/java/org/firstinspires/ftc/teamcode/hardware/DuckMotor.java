package org.firstinspires.ftc.teamcode.hardware;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class DuckMotor {

    Robot robot;
    DcMotorEx motor;
    ElapsedTime timer;

    public static double START_VELOCITY = 150;
    public static double MID_VELOCITY = 180;
    public static double END_VELOCITY = 350;

    public static double MID_DURATION = 500;
    public static double END_DURATION = 2000;
    public static double STOP_DURATION = 400;
    private boolean done;

    public static double kP = 0.018;
    public static double kI = 0.0000000000025;
    public static double kD = 0.27;
    public static double kF = 0.1;

    PIDCoefficients coefficients = new PIDCoefficients(kP,kI,kD);
    BasicPID duckVelocityControllers = new BasicPID(coefficients);

    public DuckMotor(Robot robot) {
        this.robot = robot;
        this.motor = robot.getDuck();
        timer = new ElapsedTime();
        done = false;
    }

    public void doDuckSpin() { // Make Time Based
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        new Thread(()-> {
            setVelocity(START_VELOCITY);
            sleep(MID_DURATION);
            setVelocity(MID_VELOCITY);
            sleep(END_DURATION);
            setVelocity(END_VELOCITY);
            sleep(STOP_DURATION);
            setVelocity(0);
        }).start();
    }

    public boolean isDone() {
        return done;
    }

    public void sleep(double time) {
        try {
            Thread.sleep((long) time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void setVelocity(double velo) {
        if(velo > 350) velo = 350;

        double power = -duckVelocityControllers.calculate(velo, motor.getVelocity(AngleUnit.DEGREES)) * kF;
        if(velo == 0) power = 0;

        motor.setPower(power);
    }


}
