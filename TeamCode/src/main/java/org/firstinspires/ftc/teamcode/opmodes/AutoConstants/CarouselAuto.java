package org.firstinspires.ftc.teamcode.opmodes.AutoConstants;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class CarouselAuto {
    public static class RedConstants {
        /*
            Start
         */
        public static Pose2d START_POSITION = new Pose2d(-36, -63, Math.toRadians(270));

        /*
            Location Based on level (shouldn't matter because of lift)
         */
        public static Pose2d TEAM_SHIPPING_HUB = new Pose2d(-24, -40, Math.toRadians(230)); // Level 3 & 2
        public static Pose2d TEAM_SHIPPING_HUB_LEVEL_2 = new Pose2d(-26, -41.5, Math.toRadians(230));
        public static Pose2d TEAM_SHIPPING_HUB_LEVEL_1 = new Pose2d(-28, -45, Math.toRadians(230));



        public static Pose2d CAROUSEL_POSITION_1 = new Pose2d(-63, -53, Math.toRadians(265));
        public static double forwardDistance = 0;

        /*
            Park
         */
        public static Pose2d PARK = new Pose2d(-61, -36, Math.toRadians(0));
    }

    public static class BlueConstants {
        /*
            Start
         */
        public static Pose2d START_POSITION = new Pose2d(-36, 63, Math.toRadians(90));

        /*
    Location Based on level (shouldn't matter because of lift)
 */
        public static Pose2d TEAM_SHIPPING_HUB = new Pose2d(-24, 44, Math.toRadians(122));
        public static Pose2d CAROUSEL_POSITION = new Pose2d(-52, 62, Math.toRadians(180));

        /*
            Park
         */
        public static Vector2d PARK = new Vector2d(-46, 35);
    }
}
