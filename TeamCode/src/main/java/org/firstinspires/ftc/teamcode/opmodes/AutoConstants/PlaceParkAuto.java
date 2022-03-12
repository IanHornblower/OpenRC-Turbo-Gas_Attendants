package org.firstinspires.ftc.teamcode.opmodes.AutoConstants;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class PlaceParkAuto {

    public static class RedConstants {
        /*
            Start
         */
        public static Pose2d START_POSITION = new Pose2d(12, -63, Math.toRadians(270));

        /*
            Location Based on level (shouldn't matter because of lift)
         */
        public static Pose2d TEAM_SHIPPING_HUB = new Pose2d(-2, -40, Math.toRadians(300)); // Level 3 & 2
        public static Pose2d TEAM_SHIPPING_HUB_LEVEL_2 = new Pose2d(-0.5, -41.5, Math.toRadians(300));
        public static Pose2d TEAM_SHIPPING_HUB_LEVEL_1 = new Pose2d(3, -45, Math.toRadians(300));

        /*
            Warehouse stuff
         */
        public static Pose2d OUTSIDE_WAREHOUSE = new Pose2d(10, -64.7, Math.toRadians(0));
        public static Pose2d INSIDE_WAREHOUSE = new Pose2d(45, -64.7, Math.toRadians(0));
        public static Pose2d PARKED_WAREHOUSE = new Pose2d(45, -40, Math.toRadians(90));
    }

    public static class BlueConstants {
        /*
            Start
         */
        public static Pose2d START_POSITION = new Pose2d(12, 63, Math.toRadians(90));

        /*
            Location Based on level (shouldn't matter because of lift)
         */
        public static Pose2d TEAM_SHIPPING_HUB = new Pose2d(-0.5, 41.5, Math.toRadians(50)); // Level 3 & 2
        public static Pose2d TEAM_SHIPPING_HUB_LEVEL_2 = new Pose2d(1, 43, Math.toRadians(50));
        public static Pose2d TEAM_SHIPPING_HUB_LEVEL_1 = new Pose2d(4, 46, Math.toRadians(50));                /*
            Warehouse stuff
         */
        public static Pose2d OUTSIDE_WAREHOUSE = new Pose2d(10, 64.7, Math.toRadians(0));
        public static Pose2d INSIDE_WAREHOUSE = new Pose2d(45, 64.7, Math.toRadians(0));
        public static Pose2d PARKED_WAREHOUSE = new Pose2d(45, 40, Math.toRadians(90));
    }

}