package org.firstinspires.ftc.teamcode.hardware;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.FeedforwardEx;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.NoFeedforward;
import com.ThermalEquilibrium.homeostasis.Filters.Estimators.RawValue;
import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficients;
import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficientsEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.ThermalEquilibrium.homeostasis.Systems.BasicSystem;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.noahbres.meepmeep.MeepMeep;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.function.DoubleSupplier;

import kotlin.reflect.KCallable;

@Config
public class Lift {

    // Lift - Motor Maybe 2 (encoder)
        // 2 Limit Switches

    // V4Bar - Motor (1) / Servos (2) (encoder)
        // 2 Limit Switches

    // Object Manipulator - Servo
        // Color Sensor

    Robot robot;
    DcMotorEx liftMotor;

    public static double MAX_VELO = 5;
    public static double MAX_ACCEL= 5;
    double pTime = System.currentTimeMillis();
    public double error, oVelocity, oAcceleration;

    public static double kV = 0;
    public static double kA = 0;
    public static double kS = 0;
    public static double kG = 0;
    public static double kCos = 0.15;
    public static double kP = 0.027;
    public static double kI = 0.00006;
    public static double kD = 0;

    public static double slowStart = 0.3;
    public static double maxMed = 0.3;
    public static double slowEnd = 0.3;

    PIDCoefficients coefficients = new PIDCoefficients(kP,kI,kD);
    DoubleSupplier motorPosition = new DoubleSupplier() {
        @Override
        public double getAsDouble() {
            return liftMotor.getCurrentPosition();
        }
    };
    FeedforwardCoefficientsEx feedforwardCoefficients = new FeedforwardCoefficientsEx(kV, kA, kS, kG, kCos);
    FeedforwardEx feedforward = new FeedforwardEx(feedforwardCoefficients);
    BasicPID controller = new BasicPID(coefficients);
    RawValue noFilter = new RawValue(motorPosition);
    BasicSystem system = new BasicSystem(noFilter,controller,feedforward);

    public Lift(Robot robot) {
        this.robot = robot;
        this.liftMotor = robot.getLiftMotor();
    }

    public DcMotorEx motor() {
        return liftMotor;
    }

    int state = 0;
    /*
        0 first quarter
        1 2&3 quarter
        2 last quarter
        3 super close
     */
    public void setPosition(double position) {
        error = (position - liftMotor.getCurrentPosition());
        if(Math.abs(error) >= position*(3.0/4.0)) {
            state = 0;
        }
        if(Math.abs(error) > position/4 && Math.abs(error) < position*(3.0/4.0)) {
            state = 1;
        }
        else if(Math.abs(error) <= position/4 && Math.abs(error) >= position/15) {
            state = 2;
        }
        else if(Math.abs(error) <= position/15) {
            state = 3;
        }

        double angle = robot.lift.tickToRadians(position) + Math.toRadians(44.3);
        double output = -controller.calculate(position, liftMotor.getCurrentPosition());


        switch (state) {
            case 0:
                liftMotor.setPower(output * slowStart);
                break;
            case 1:
                liftMotor.setPower(output * maxMed);
                break;
            case 2:
                liftMotor.setPower(output * slowEnd);
                break;
            case 3:
                liftMotor.setPower(output * slowEnd);
                break;
        }

    }

    public int getState() {
        return state;
    }

    public void setPositionTrap(double position) {
        double cVelocity = liftMotor.getVelocity();
        double cTime = System.currentTimeMillis();

        error = (position - liftMotor.getCurrentPosition());

        double dMult = 1.0;

        if(error < 0) dMult = -1.0;
        else dMult = 1.0;

        if (MAX_VELO > Math.abs(cVelocity)) {
            oVelocity = cVelocity + dMult * MAX_ACCEL * (cTime - pTime);
            oAcceleration = MAX_ACCEL;
        }
        else {
            oVelocity = MAX_VELO;
            oAcceleration = 0;
        }

        if(error > error/2) {
            oVelocity = MAX_VELO/ (1 - (1/error));
            oAcceleration = -MAX_ACCEL;
        }

        liftMotor.setPower(0);

        pTime = cTime;
    }

    public void setVelocity(double velocity) {
        liftMotor.setVelocity(velocity);
    }

    public double tickToRadians(double ticks) {
        return (Math.PI * 2.0/537.6) * (ticks);
    }




}
