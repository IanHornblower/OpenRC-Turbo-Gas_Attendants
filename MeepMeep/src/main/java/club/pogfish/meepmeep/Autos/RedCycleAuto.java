package club.pogfish.meepmeep.Autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import club.pogfish.meepmeep.constants.PlaceParkAuto;
import club.pogfish.meepmeep.constants.WarehouseCycleAuto;

public class RedCycleAuto {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                //.setConstraints(36, 40, Math.toRadians(120), Math.toRadians(78), 10) // Slowest Speed Current DT can drive should be 20-30% faster
                .setConstraints(52.48291908330528, 52.48291908330528, Math.toRadians(250.58748), Math.toRadians(250.58748), 10)
                .setDimensions(12, 18)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(WarehouseCycleAuto.RedConstants.ALT_START_POSITION)
                                /*
                                    Drop Block
                                 */

                                /*  // Spline to Hub kinda slower gotta test. //

                                .setReversed(true)

                                .splineTo(
                                       WarehouseCycleAuto.RedConstants.TEAM_SHIPPING_HUB.vec(),
                                       Math.toRadians(112))

                                .setReversed(false)

                                 */

                                .addTemporalMarker(0.2, ()-> {
                                    // Drop Intake
                                    // Set Lift & V4B to correct level
                                })
                                .addTemporalMarker(1, ()-> {  // Tune this time level
                                    // Drop Preload block
                                })

                                .lineToSplineHeading(WarehouseCycleAuto.RedConstants.TEAM_SHIPPING_HUB)

                                .addTemporalMarker(()-> {
                                    // Return Lift System to intaking position
                                })
                                .setReversed(false)

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

                                .splineTo(
                                        WarehouseCycleAuto.RedConstants.OUTSIDE_WAREHOUSE.vec(),
                                        Math.toRadians(0))
                                .lineToConstantHeading(WarehouseCycleAuto.RedConstants.INSIDE_WAREHOUSE.vec())
                                .lineToConstantHeading(WarehouseCycleAuto.RedConstants.OUTSIDE_WAREHOUSE.vec())
                                .splineTo(
                                        WarehouseCycleAuto.RedConstants.CYCLE_SHIPPING_HUB.vec(),
                                        WarehouseCycleAuto.RedConstants.CYCLE_SHIPPING_HUB.getHeading())
                                .setTangent(WarehouseCycleAuto.RedConstants.TANGENT)

                                // Cycle 6

                                .splineTo(
                                        WarehouseCycleAuto.RedConstants.OUTSIDE_WAREHOUSE.vec(),
                                        Math.toRadians(0))
                                .lineToConstantHeading(WarehouseCycleAuto.RedConstants.INSIDE_WAREHOUSE.vec())
                                .lineToConstantHeading(WarehouseCycleAuto.RedConstants.OUTSIDE_WAREHOUSE.vec())
                                .splineTo(
                                        WarehouseCycleAuto.RedConstants.CYCLE_SHIPPING_HUB.vec(),
                                        WarehouseCycleAuto.RedConstants.CYCLE_SHIPPING_HUB.getHeading())
                                .setTangent(WarehouseCycleAuto.RedConstants.TANGENT)

                                .build() // End Auto [stop();]
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
