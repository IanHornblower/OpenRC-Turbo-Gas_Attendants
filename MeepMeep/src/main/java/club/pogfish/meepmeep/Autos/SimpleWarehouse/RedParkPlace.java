package club.pogfish.meepmeep.Autos.SimpleWarehouse;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import club.pogfish.meepmeep.constants.PlaceParkAuto;

public class RedParkPlace {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(150), Math.toRadians(150), 9)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(PlaceParkAuto.RedConstants.START_POSITION)
                                .lineToLinearHeading(PlaceParkAuto.RedConstants.TEAM_SHIPPING_HUB)
                                .lineToLinearHeading(PlaceParkAuto.RedConstants.OUTSIDE_WAREHOUSE)
                                .lineToConstantHeading(PlaceParkAuto.RedConstants.INSIDE_WAREHOUSE.vec())
                                .lineToConstantHeading(PlaceParkAuto.RedConstants.PARKED_WAREHOUSE.vec())
                                .build() // End Auto [stop();]
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
