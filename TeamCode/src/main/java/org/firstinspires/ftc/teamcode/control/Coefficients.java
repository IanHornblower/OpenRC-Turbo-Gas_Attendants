package org.firstinspires.ftc.teamcode.control;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
public class Coefficients {

    public static double turnKp = 1;
    public static double turnKi = 0;
    public static double turnKd = 0;

    PIDCoefficients turnPID = new PIDCoefficients(turnKp, turnKi, turnKd);

    BasicPID turnPIDBasic = new BasicPID(turnPID);

    public AngleController turn = new AngleController(turnPIDBasic);

    public static double xKp = 0.1;
    public static double xKi = 0;
    public static double xKd = 0;
    public static double xMaxIntegralSum = 0;
    public static double xStabilityThreshold = 0;
    public static double xLowPassGain = 0;

    PIDCoefficientsEx xPID = new PIDCoefficientsEx(xKp, xKi, xKd,
            xMaxIntegralSum, xStabilityThreshold, xLowPassGain);

    public PIDEx x = new PIDEx(xPID);

    public static double yKp = 0.1;
    public static double yKi = 0;
    public static double yKd = 0;
    public static double yMaxIntegralSum = 0;
    public static double yStabilityThreshold = 0;
    public static double yLowPassGain = 0;

    PIDCoefficientsEx yPID = new PIDCoefficientsEx(yKp, yKi, yKd,
            yMaxIntegralSum, yStabilityThreshold, yLowPassGain);

    public PIDEx y = new PIDEx(yPID);

    public static double distanceKp = 0.1;
    public static double distanceKi = 0;
    public static double distanceKd = 0;
    public static double distanceMaxIntegralSum = 0;
    public static double distanceStabilityThreshold = 0;
    public static double distanceLowPassGain = 0;

    PIDCoefficientsEx distancePID = new PIDCoefficientsEx(distanceKp, distanceKi, distanceKd,
            distanceMaxIntegralSum, distanceStabilityThreshold, distanceLowPassGain);

    public PIDEx distance = new PIDEx(distancePID);

    public static double headingKp = 0.4;
    public static double headingKi = 0;
    public static double headingKd = 0;

    PIDCoefficients headingPID = new PIDCoefficients(headingKp, headingKi, headingKd);

    BasicPID headingBasicPID = new BasicPID(headingPID);

    public AngleController heading = new AngleController(headingBasicPID);

    public static double differentialTurnKp = 0;
    public static double differentialTurnKi = 0;
    public static double differentialTurnKd = 0;

    PIDCoefficients differentialTurnPID = new PIDCoefficients(differentialTurnKp, differentialTurnKi, differentialTurnKd);

    BasicPID differentialTurnBasicPID = new BasicPID(differentialTurnPID);

    public AngleController differentialTurn = new AngleController(differentialTurnBasicPID);

    public static double differentialForwardKp = 0;
    public static double differentialForwardKi = 0;
    public static double differentialForwardKd = 0;
    public static double differentialForwardMaxIntegralSum = 0;
    public static double differentialForwardStabilityThreshold = 0;
    public static double differentialForwardLowPassGain = 0;

    PIDCoefficientsEx differentialForwardPID = new PIDCoefficientsEx(differentialForwardKp, differentialForwardKi, differentialForwardKd,
            differentialForwardMaxIntegralSum, differentialForwardStabilityThreshold, differentialForwardLowPassGain);

    public PIDEx differentialForward = new PIDEx(differentialForwardPID);

    public static double liftKp = 0.08;
    public static double liftKi = 0.0;
    public static double liftKd = 0.0;
    public static double liftKf = 0.08  ;
    public static double liftMaxIntegralSum = 0;
    public static double liftStabilityThreshold = 0;
    public static double liftLowPassGain = 0;

    PIDCoefficientsEx liftPID = new PIDCoefficientsEx(liftKp, liftKi, liftKd,
            liftMaxIntegralSum, liftStabilityThreshold, liftLowPassGain);

    public PIDEx lift = new PIDEx(liftPID);

    public Coefficients() {

    }

    public void update() {
        // Lift

        liftPID = new PIDCoefficientsEx(liftKp, liftKi, liftKd,
                liftMaxIntegralSum, liftStabilityThreshold, liftLowPassGain);

        lift = new PIDEx(liftPID);

        // Diffy Power

        differentialForwardPID = new PIDCoefficientsEx(differentialForwardKp, differentialForwardKi, differentialForwardKd,
                differentialForwardMaxIntegralSum, differentialForwardStabilityThreshold, differentialForwardLowPassGain);

        differentialForward = new PIDEx(differentialForwardPID);

        // Diffy Turn

        differentialTurnPID = new PIDCoefficients(differentialTurnKp, differentialTurnKi, differentialTurnKd);

        differentialTurnBasicPID = new BasicPID(differentialTurnPID);

        differentialTurn = new AngleController(differentialTurnBasicPID);

        // Heading

        headingPID = new PIDCoefficients(headingKp, headingKi, headingKd);

        headingBasicPID = new BasicPID(headingPID);

        AngleController heading = new AngleController(headingBasicPID);

        // Y

        yPID = new PIDCoefficientsEx(yKp, yKi, yKd,
                yMaxIntegralSum, yStabilityThreshold, yLowPassGain);

        y = new PIDEx(yPID);

        // X

        xPID = new PIDCoefficientsEx(xKp, xKi, xKd,
                xMaxIntegralSum, xStabilityThreshold, xLowPassGain);

        x = new PIDEx(xPID);

        // Turn

        turnPID = new PIDCoefficients(turnKp, turnKi, turnKd);

        turnPIDBasic = new BasicPID(turnPID);

        turn = new AngleController(turnPIDBasic);
    }
}
