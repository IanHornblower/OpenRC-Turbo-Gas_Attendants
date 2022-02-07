package org.firstinspires.ftc.teamcode.opmodes.Comp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.control.*;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.opmodes.Comp.MatchConfig;
import org.firstinspires.ftc.teamcode.util.AngleUtil;

import java.util.ArrayList;

@Disabled
@Autonomous(name = "Switcher", group = "Comp")
public class Switcher extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {



        while(!opModeIsActive() && !isStopRequested()) {
            telemetry.addLine("Press Cross for Blue and Circle Red");
            telemetry.addLine("Press Triangle for Warehouse and Square for Storage");
            telemetry.addData("Current Side", MatchConfig.side.toString());
            telemetry.addData("Current Type", MatchConfig.park.toString());
            telemetry.update();

            if(gamepad1.square || gamepad2.square) {
                MatchConfig.park = MatchConfig.PARK.STORAGE;
            }
            if(gamepad1.triangle || gamepad2.triangle) {
                MatchConfig.park = MatchConfig.PARK.WAREHOUSE;
            }
            if(gamepad1.circle || gamepad2.circle) {
                MatchConfig.side = MatchConfig.SIDE.RED;
            }
            if(gamepad1.cross || gamepad2.cross) {
                MatchConfig.side = MatchConfig.SIDE.BLUE;
            }
        }


        waitForStart();

        while(opModeIsActive()) {

            stop();
        }
    }
}
