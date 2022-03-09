package club.pogfish.meepmeep.Autos.Carousel;

import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import club.pogfish.meepmeep.constants.CarouselAuto;
import club.pogfish.meepmeep.constants.PlaceParkAuto;

public class RedCarouselAuto {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(150), Math.toRadians(150), 9)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(CarouselAuto.RedConstants.START_POSITION)
                                .lineToLinearHeading(   CarouselAuto.RedConstants.TEAM_SHIPPING_HUB)
                                .lineToLinearHeading(   CarouselAuto.RedConstants.CAROUSEL_POSITION).setReversed(true)
                                .splineTo(CarouselAuto.RedConstants.PARK, Math.toRadians(180))
                                .back(14)
                                .build() // End Auto [stop();]
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
