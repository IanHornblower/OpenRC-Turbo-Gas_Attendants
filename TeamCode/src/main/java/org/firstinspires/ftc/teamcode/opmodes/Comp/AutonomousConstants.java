package org.firstinspires.ftc.teamcode.opmodes.Comp;

import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.AngleUtil;

public class AutonomousConstants {
    public static class RedConstants {
        public static class Warehouse {
            public static Pose2D START_POSITION = new Pose2D(63, 6.5, AngleUtil.interpretAngle(0));

            public static Pose2D SHIPPING_HUB_LEVEL_ONE = new Pose2D(45.5, -5, Math.toRadians(20));
            public static Pose2D SHIPPING_HUB_LEVEL_TWO = new Pose2D(45.5, -5, Math.toRadians(20));
            public static Pose2D SHIPPING_HUB_LEVEL_THREE = new Pose2D(45.5, -5, Math.toRadians(20));

            public static Pose2D WAREHOUSE_WALL = new Pose2D(64.5, 7, Math.toRadians(90));
            public static double DISTANCE_BLIND = 12;

        }
    }

    public static class BlueConstants {
        public static class Warehouse {

        }

    }



}
