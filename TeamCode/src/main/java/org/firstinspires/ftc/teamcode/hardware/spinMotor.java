package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
public class spinMotor {

    Robot robot;

    DcMotor duck;

    public static double speed = 0.74;

    public static double leftMult = -1;
    public static double rightMult = -1;

    public spinMotor(Robot robot) {
        this.robot = robot;
        this.duck = robot.getDuck();
    }

    public void run(boolean left, boolean right) {
        if(left) {
            duck.setPower(-0.5);
        }
        else if (right) {
            duck.setPower(0.5);
        }
        else {
            duck.setPower(0);
        }
    }

}
