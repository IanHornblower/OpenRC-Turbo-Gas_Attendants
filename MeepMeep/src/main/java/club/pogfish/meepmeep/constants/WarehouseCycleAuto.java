package club.pogfish.meepmeep.constants;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

public class WarehouseCycleAuto {

    public static class RedConstants {
        /*
            Start
         */
        public static Pose2d START_POSITION = new Pose2d(9, -65, Math.toRadians(0));

        /*
            Location Based on level (shouldn't matter because of lift)
         */
        public static Pose2d TEAM_SHIPPING_HUB = new Pose2d(-3, -44, Math.toRadians(292));

        public static Pose2d CYCLE_SHIPPING_HUB = new Pose2d(-3, -44, Math.toRadians(112)); // Flipped across X-Axis (normally 292)

        /*
            Warehouse stuff
         */
        public static Pose2d OUTSIDE_WAREHOUSE = new Pose2d(20, -63.7, Math.toRadians(0));
        public static Pose2d INSIDE_WAREHOUSE = new Pose2d(46, -63.7, Math.toRadians(0));

        public static double TANGENT = Math.toRadians(292);
        /*
            Parking
         */
        public static Pose2d PARKED_WAREHOUSE = new Pose2d(40, -40, Math.toRadians(90));
    }

    public static class BlueConstants {

    }

}
