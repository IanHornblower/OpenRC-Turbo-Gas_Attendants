package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.opmodes.AutoConstants.WarehouseCycleAuto;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous
public class Auto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        robot.driveController.setGamepad(gamepad1);
        robot.operatorController.setGamepad(gamepad2);

        robot.driveTrain.setPoseEstimate(WarehouseCycleAuto.RedConstants.START_POSITION);

        while (!opModeIsActive() && !isStopRequested()) {

        }

        waitForStart();
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && !isStopRequested()) {
            double leftX = -robot.driveController.leftJoystick.x();
            double leftY = -robot.driveController.leftJoystick.y();
            double turn =  -robot.driveController.rightJoystick.x()*0.8;

            telemetry.addData("Timer", timer.seconds());
            telemetry.update();

             TrajectorySequence auto = robot.driveTrain.trajectorySequenceBuilder(WarehouseCycleAuto.RedConstants.ALT_START_POSITION)
                    /*
                        Drop Block
                     */

                     // Spline to Hub kinda slower gotta test. //

                     .lineToSplineHeading(WarehouseCycleAuto.RedConstants.TEAM_SHIPPING_HUB)

                    .setReversed(false)

                    .addTemporalMarker(0.2, ()-> {
                        // Drop Intake
                        // Set Lift & V4B to correct level
                    })
                    .addTemporalMarker(1, ()-> {  // Tune this time level
                        // Drop Preload block
                    })

                    //.lineToSplineHeading(WarehouseCycleAuto.RedConstants.TEAM_SHIPPING_HUB)

                    .addTemporalMarker(()-> {
                        // Return Lift System to intaking position
                    })

                    /*
                        Start Cycles
                     */

                    // Cycle 1


                    .splineTo(
                            WarehouseCycleAuto.RedConstants.OUTSIDE_WAREHOUSE.vec(),
                            Math.toRadians(0))
                    .lineToConstantHeading(WarehouseCycleAuto.RedConstants.INSIDE_WAREHOUSE.vec())
                    .lineToConstantHeading(WarehouseCycleAuto.RedConstants.OUTSIDE_WAREHOUSE.vec())
                    .splineTo(
                            WarehouseCycleAuto.RedConstants.CYCLE_SHIPPING_HUB.vec(),
                            WarehouseCycleAuto.RedConstants.CYCLE_SHIPPING_HUB.getHeading())
                    .setTangent(WarehouseCycleAuto.RedConstants.TANGENT)

                    // Cycle 2

                    .splineTo(
                            WarehouseCycleAuto.RedConstants.OUTSIDE_WAREHOUSE.vec(),
                            Math.toRadians(0))
                    .lineToConstantHeading(WarehouseCycleAuto.RedConstants.INSIDE_WAREHOUSE.vec())
                    .lineToConstantHeading(WarehouseCycleAuto.RedConstants.OUTSIDE_WAREHOUSE.vec())
                    .splineTo(
                            WarehouseCycleAuto.RedConstants.CYCLE_SHIPPING_HUB.vec(),
                            WarehouseCycleAuto.RedConstants.CYCLE_SHIPPING_HUB.getHeading())
                    .setTangent(WarehouseCycleAuto.RedConstants.TANGENT)

                    // Cycle 3

                    .splineTo(
                            WarehouseCycleAuto.RedConstants.OUTSIDE_WAREHOUSE.vec(),
                            Math.toRadians(0))
                    .lineToConstantHeading(WarehouseCycleAuto.RedConstants.INSIDE_WAREHOUSE.vec())
                    .lineToConstantHeading(WarehouseCycleAuto.RedConstants.OUTSIDE_WAREHOUSE.vec())
                    .splineTo(
                            WarehouseCycleAuto.RedConstants.CYCLE_SHIPPING_HUB.vec(),
                            WarehouseCycleAuto.RedConstants.CYCLE_SHIPPING_HUB.getHeading())
                    .setTangent(WarehouseCycleAuto.RedConstants.TANGENT)

                    // Cycle 4

                    .splineTo(
                            WarehouseCycleAuto.RedConstants.OUTSIDE_WAREHOUSE.vec(),
                            Math.toRadians(0))
                    .lineToConstantHeading(WarehouseCycleAuto.RedConstants.INSIDE_WAREHOUSE.vec())
                    .lineToConstantHeading(WarehouseCycleAuto.RedConstants.OUTSIDE_WAREHOUSE.vec())
                    .splineTo(
                            WarehouseCycleAuto.RedConstants.CYCLE_SHIPPING_HUB.vec(),
                            WarehouseCycleAuto.RedConstants.CYCLE_SHIPPING_HUB.getHeading())
                    .setTangent(WarehouseCycleAuto.RedConstants.TANGENT)

                    // Cycle 5

                    //.splineTo(
                    //        WarehouseCycleAuto.RedConstants.OUTSIDE_WAREHOUSE.vec(),
                    //        Math.toRadians(0))
                    //.lineToConstantHeading(WarehouseCycleAuto.RedConstants.INSIDE_WAREHOUSE.vec())
                    //.lineToConstantHeading(WarehouseCycleAuto.RedConstants.OUTSIDE_WAREHOUSE.vec())
                    //.splineTo(
                    //        WarehouseCycleAuto.RedConstants.CYCLE_SHIPPING_HUB.vec(),
                    //        WarehouseCycleAuto.RedConstants.CYCLE_SHIPPING_HUB.getHeading())
                    //.setTangent(WarehouseCycleAuto.RedConstants.TANGENT)

                     .splineTo(
                             WarehouseCycleAuto.RedConstants.OUTSIDE_WAREHOUSE.vec(),
                             Math.toRadians(0))
                     .lineToConstantHeading(WarehouseCycleAuto.RedConstants.INSIDE_WAREHOUSE.vec())

                    .build(); // End Auto [stop();]

            telemetry.addData("Timer", timer.seconds());
            telemetry.update();
            robot.driveTrain.followTrajectorySequence(auto);
            telemetry.addLine("DONE");
            telemetry.update();
            stop();

        }
    }
}
