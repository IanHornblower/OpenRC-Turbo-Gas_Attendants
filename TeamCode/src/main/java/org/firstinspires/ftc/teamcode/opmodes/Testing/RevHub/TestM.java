package org.firstinspires.ftc.teamcode.opmodes.Testing.RevHub;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@Config
@TeleOp(name = "Tests Lmao", group = "Testing")
public class TestM extends LinearOpMode {

    DcMotor Motor;
    Servo Servo;

    @Override
    public void runOpMode() {

        Motor = hardwareMap.get(DcMotor.class, "motor");
        Servo = hardwareMap.get(Servo.class, "servo");

        waitForStart();
        while(opModeIsActive()) {

            Motor.setPower(gamepad1.left_stick_y);

            if (gamepad1.right_stick_y > 0.5)
                Servo.setPosition(Servo.getPosition() + 0.25);
            else if (gamepad1.right_stick_y < -0.5)
                Servo.setPosition(Servo.getPosition() - 0.25);

        }
    }
}
