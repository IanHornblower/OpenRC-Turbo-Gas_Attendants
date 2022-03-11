package club.pogfish.meepmeep.Autos.CycleWarehouse;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import club.pogfish.meepmeep.constants.PlaceParkAuto;
import club.pogfish.meepmeep.constants.WarehouseCycleAuto;

public class SimpleRedCycleAuto {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                //.setConstraints(36, 40, Math.toRadians(120), Math.toRadians(78), 10) // Slowest Speed Current DT can drive should be 20-30% faster
                .setConstraints(42, 52, Math.toRadians(250.58748), Math.toRadians(250.58748), 10)
                .setDimensions(12, 18)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(WarehouseCycleAuto.RedConstants.ALT_START_POSITION)
                                /*
                                    Reverse  & stop intakes
                                    TODO: THESE WILL HAVE TO BE TUNED
                                 */

                                .addTemporalMarker(6.5, ()-> {
                                    // reverse intake
                                })
                                .addTemporalMarker(7.2, ()-> {
                                    // stop intake
                                })

                                .addTemporalMarker(13.85, ()-> {
                                    // reverse intake
                                })
                                .addTemporalMarker(15.17, ()-> {
                                    // stop intake
                                })


                                /*
                                    Drop Block of and start cycle
                                 */

                                .addTemporalMarker(()-> {
                                    // Set Lift to right level
                                })
                                .lineToLinearHeading(WarehouseCycleAuto.RedConstants.TEAM_SHIPPING_HUB)
                                .addTemporalMarker(()-> {
                                    // drop box
                                })
                                .waitSeconds(1.5)
                                .lineToLinearHeading(WarehouseCycleAuto.RedConstants.OUTSIDE_WAREHOUSE)
                                .addTemporalMarker(()-> {
                                    // drop intake
                                    // and start
                                })


                                /*
                                    Start Cycle
                                 */

                                // Cycle 1

                                .forward(35)
                                .lineToLinearHeading(WarehouseCycleAuto.RedConstants.OUTSIDE_WAREHOUSE)
                                .addTemporalMarker(()-> {
                                    // set lift 3
                                })
                                .lineToLinearHeading(WarehouseCycleAuto.RedConstants.TEAM_SHIPPING_HUB)
                                .addTemporalMarker(()-> {
                                    // drop box
                                })
                                .waitSeconds(1.5)
                                .addTemporalMarker(()-> {
                                    // return lift
                                })
                                .lineToLinearHeading(WarehouseCycleAuto.RedConstants.OUTSIDE_WAREHOUSE)
                                .addTemporalMarker(()-> {
                                    // start intake
                                })

                                // Cycle 2

                                .forward(35)
                                .lineToLinearHeading(WarehouseCycleAuto.RedConstants.OUTSIDE_WAREHOUSE)
                                .addTemporalMarker(()-> {
                                    // Stop intake
                                })
                                .addTemporalMarker(()-> {
                                    // set lift 3
                                })
                                .lineToLinearHeading(WarehouseCycleAuto.RedConstants.TEAM_SHIPPING_HUB)
                                .addTemporalMarker(()-> {
                                    // drop box
                                })
                                .waitSeconds(1.5)
                                .addTemporalMarker(()-> {
                                    // return lift
                                })
                                .lineToLinearHeading(WarehouseCycleAuto.RedConstants.OUTSIDE_WAREHOUSE)
                                .addTemporalMarker(()-> {
                                    // start intake
                                })

                                // Cycle 3

                                .forward(35)
                                .lineToLinearHeading(WarehouseCycleAuto.RedConstants.OUTSIDE_WAREHOUSE)
                                .addTemporalMarker(()-> {
                                    // Stop intake
                                })
                                .addTemporalMarker(()-> {
                                    // set lift 3
                                })
                                .lineToLinearHeading(WarehouseCycleAuto.RedConstants.TEAM_SHIPPING_HUB)
                                .addTemporalMarker(()-> {
                                    // drop box
                                })
                                .waitSeconds(1.5)
                                .addTemporalMarker(()-> {
                                    // return lift
                                })
                                .lineToLinearHeading(WarehouseCycleAuto.RedConstants.OUTSIDE_WAREHOUSE)

                                // park
                                .forward(32)

                                .build() // End Auto [stop();]
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
