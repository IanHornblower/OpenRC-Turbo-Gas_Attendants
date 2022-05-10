package org.firstinspires.ftc.teamcode.opmodes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.opmodes.AutoConstants.PlaceParkAuto;
import org.firstinspires.ftc.teamcode.opmodes.MatchConfig;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.FreightFrenzyCamera;

@Autonomous
public class RedCycleAuto extends LinearOpMode {

    enum ElementPosition {
        LEFT, MIDDLE, RIGHT, NONE
    }

    double liftPosition = 0;
    double TSELiftPosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Starting Hardware Map & Camera");
        telemetry.update();

        MatchConfig.side = MatchConfig.SIDE.RED;  // Change for blue
        MatchConfig.park = MatchConfig.PARK.WAREHOUSE;

        Robot robot = new Robot(hardwareMap);
        robot.driveController.setGamepad(gamepad1);
        robot.operatorController.setGamepad(gamepad2);

        FreightFrenzyCamera camera = new FreightFrenzyCamera(hardwareMap);

        robot.driveTrain.setPoseEstimate(PlaceParkAuto.RedConstants.START_POSITION);
        robot.lift.startServo();
        //robot.freightDetector.run();

        camera.initWebCamera();
        camera.initPipeline();
        camera.startCamera();

        camera.startCameraStream(0);
        CameraStreamServer.getInstance().setSource(camera.getCamera());

        ElementPosition state = null;

        // While Loop

        while(!opModeIsActive() && !isStopRequested()) {

            switch (camera.determinePosition()) {
                case A:
                    state = ElementPosition.LEFT;
                    break;
                case B:
                    state = ElementPosition.MIDDLE;
                    break;
                case C:
                    state = ElementPosition.RIGHT;
                    break;
                case ABSENT: // Account for displacement
                    state = ElementPosition.RIGHT;
                    break;
            }

            telemetry.clear();
            telemetry.addLine("Waiting For Start");
            telemetry.addLine(
                    "Autonomous Configuration: \n" +
                            "Side: " + MatchConfig.side.toString() +
                            "\nParking Location: " + MatchConfig.park.toString());
            telemetry.addData("Location", camera.sDeterminePosition());
            telemetry.update();
        }

        TrajectorySequence autoLeft = robot.driveTrain.trajectorySequenceBuilder(PlaceParkAuto.RedConstants.START_POSITION)
                .addTemporalMarker(()-> {
                    robot.intakeSys.regularFreightIntake();
                })
                .addTemporalMarker(0.5, ()-> {
                    liftPosition = 1400;
                })
                .lineToLinearHeading(PlaceParkAuto.RedConstants.TEAM_SHIPPING_HUB_LEVEL_1)
                .addTemporalMarker(()-> {
                    robot.lift.superDrop();
                })
                .waitSeconds(2)
                .addTemporalMarker(()-> {
                    robot.lift.drivingServo();
                })
                .lineToLinearHeading(PlaceParkAuto.RedConstants.OUTSIDE_WAREHOUSE)
                .addTemporalMarker(()-> {
                    liftPosition = 0;
                })
                .lineToConstantHeading(PlaceParkAuto.RedConstants.INSIDE_WAREHOUSE.vec())
                // add stuff here
                .lineToConstantHeading(PlaceParkAuto.RedConstants.PARKED_WAREHOUSE.vec())


                .build();

        TrajectorySequence autoMiddle = robot.driveTrain.trajectorySequenceBuilder(PlaceParkAuto.RedConstants.START_POSITION)
                .addTemporalMarker(()-> {
                    robot.intakeSys.regularFreightIntake();
                })
                .addTemporalMarker(0.5, ()-> {
                    liftPosition = 1400;
                })
                .lineToLinearHeading(PlaceParkAuto.RedConstants.TEAM_SHIPPING_HUB_LEVEL_2)
                .addTemporalMarker(()-> {
                    robot.lift.superDrop();
                })
                .waitSeconds(2)
                .addTemporalMarker(()-> {
                    robot.lift.drivingServo();
                })
                .lineToLinearHeading(PlaceParkAuto.RedConstants.OUTSIDE_WAREHOUSE)
                .addTemporalMarker(()-> {
                    liftPosition = 0;
                })
                .lineToConstantHeading(PlaceParkAuto.RedConstants.INSIDE_WAREHOUSE.vec())
                .lineToConstantHeading(PlaceParkAuto.RedConstants.PARKED_WAREHOUSE.vec())

                .build();

        TrajectorySequence autoRight = robot.driveTrain.trajectorySequenceBuilder(PlaceParkAuto.RedConstants.START_POSITION)
                .addTemporalMarker(()-> {
                    robot.intakeSys.regularFreightIntake();
                })
                .addTemporalMarker(0.5, ()-> {
                    liftPosition = 1400;
                })
                .lineToLinearHeading(PlaceParkAuto.RedConstants.TEAM_SHIPPING_HUB)
                .addTemporalMarker(()-> {
                    robot.lift.drop();
                })
                .waitSeconds(2)
                .addTemporalMarker(()-> {
                    robot.lift.drivingServo();
                })
                .lineToLinearHeading(PlaceParkAuto.RedConstants.OUTSIDE_WAREHOUSE)
                .addTemporalMarker(()-> {
                    liftPosition = 0;
                })
                .lineToConstantHeading(PlaceParkAuto.RedConstants.INSIDE_WAREHOUSE.vec())
                .lineToConstantHeading(PlaceParkAuto.RedConstants.PARKED_WAREHOUSE.vec())

                .build();

        /*
            Trajectory thingy
         */

        switch (state) {
            case LEFT:
                robot.driveTrain.followTrajectorySequenceAsync(autoLeft);
                break;
            case MIDDLE:
                robot.driveTrain.followTrajectorySequenceAsync(autoMiddle);
                break;
            case RIGHT:
                robot.driveTrain.followTrajectorySequenceAsync(autoRight);
                break;
            default: robot.driveTrain.followTrajectorySequenceAsync(autoRight);
        }


        telemetry.clear();
        telemetry.update();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            robot.driveTrain.update();

            robot.lift.setPosition(liftPosition);

            if(!robot.driveTrain.isBusy()) {
                stop();
            }
        }
    }
}
