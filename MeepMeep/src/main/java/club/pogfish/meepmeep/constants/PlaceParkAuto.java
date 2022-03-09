package club.pogfish.meepmeep.constants;

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
        public static Pose2d TEAM_SHIPPING_HUB = new Pose2d(-3, -44, Math.toRadians(292));

        /*
            Warehouse stuff
         */
        public static Pose2d OUTSIDE_WAREHOUSE = new Pose2d(10, -63.7, Math.toRadians(0));
        public static Pose2d INSIDE_WAREHOUSE = new Pose2d(40, -63.7, Math.toRadians(0));
        public static Pose2d PARKED_WAREHOUSE = new Pose2d(40, -40, Math.toRadians(90));
    }

    public static class BlueConstants {
        /*
            Start
         */
        public static Pose2d START_POSITION = new Pose2d(12, 63, Math.toRadians(90));

        /*
            Location Based on level (shouldn't matter because of lift)
         */
        public static Pose2d TEAM_SHIPPING_HUB = new Pose2d(-3, 44, Math.toRadians(68));

        /*
            Warehouse stuff
         */
        public static Pose2d OUTSIDE_WAREHOUSE = new Pose2d(10, 63.7, Math.toRadians(0));
        public static Pose2d INSIDE_WAREHOUSE = new Pose2d(40, 63.7, Math.toRadians(0));
        public static Pose2d PARKED_WAREHOUSE = new Pose2d(40, 40, Math.toRadians(270));
    }

}