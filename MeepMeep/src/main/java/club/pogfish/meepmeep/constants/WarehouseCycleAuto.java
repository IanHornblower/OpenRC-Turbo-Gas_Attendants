package club.pogfish.meepmeep.constants;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

public class WarehouseCycleAuto {

    public static class RedConstants {
        /*
            Start
         */
        public static Pose2d ALT_START_POSITION = new Pose2d(9, -63, Math.toRadians(270));

        /*
            Location Based on level (shouldn't matter because of lift)
         */
        public static Pose2d TEAM_SHIPPING_HUB = new Pose2d(-2.5, -39.5, Math.toRadians(300));

        /*
            Warehouse stuff
         */
        public static Pose2d OUTSIDE_WAREHOUSE = new Pose2d(10, -63.7, Math.toRadians(0));
        public static Pose2d INSIDE_WAREHOUSE = new Pose2d(50, -63.7, Math.toRadians(0)); // Maybe use prolly not

        /*
            Parking
         */
        public static Pose2d PARKED_WAREHOUSE = new Pose2d(40, -40, Math.toRadians(90)); // Maybe park inside
    }

    public static class BlueConstants {
        /*
            Start
         */
        public static Pose2d ALT_START_POSITION = new Pose2d(9, 63, Math.toRadians(90));

        /*
            Location Based on level (shouldn't matter because of lift)
         */
        public static Pose2d TEAM_SHIPPING_HUB = new Pose2d(-2.5, 39.5, Math.toRadians(60));

        /*
            Warehouse stuff
         */
        public static Pose2d OUTSIDE_WAREHOUSE = new Pose2d(10, 63.7, Math.toRadians(0));
        public static Pose2d INSIDE_WAREHOUSE = new Pose2d(50, 63.7, Math.toRadians(0)); // Maybe use prolly not

        /*
            Parking
         */
        public static Pose2d PARKED_WAREHOUSE = new Pose2d(40, 40, Math.toRadians(90)); // Maybe park inside
    }

}
