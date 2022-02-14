package org.firstinspires.ftc.teamcode.dashboard;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.Array;

public class Field {

    TelemetryPacket packet;
    Canvas field;

    // All Robots
    String color = "goldenrod";
    int strokeWidth = 1;

    // Standard Robot
    double robotWidth = 10;
    double robotHeight = 18;

    // Circular Robot
    double radius = 9;

    public Field (TelemetryPacket packet) {
        this.packet = packet;
    }

    private double[][] createRobotRectangle(Pose2D pos) {
        double[] x = {};
        double[] y = {};

        double hypot = Math.hypot(robotWidth/2, robotHeight/2);
        double theta = Math.atan2(robotHeight, robotWidth);

        // X Points

        x = Array.appendArray(x,
                pos.x + hypot * Math.cos(pos.heading+theta));

        x = Array.appendArray(x,
                pos.x + hypot * Math.cos(pos.heading+(2*theta)));

        x = Array.appendArray(x,
                pos.x + hypot * Math.cos(pos.heading-(2*theta)));

        x = Array.appendArray(x,
                pos.x + hypot * Math.cos(pos.heading-theta));

        // Y Points

        y = Array.appendArray(y,
                pos.y + hypot * Math.sin(pos.heading+theta));

        y = Array.appendArray(y,
                pos.y + hypot * Math.sin(pos.heading+(2*theta)));

        y = Array.appendArray(y,
                pos.y + hypot * Math.sin(pos.heading-(2*theta)));

        y = Array.appendArray(y,
                pos.y + hypot * Math.sin(pos.heading-theta));

        return new double[][] {x, y};
    }




    public void createRobot(Pose2D pos) {
        double[][] xy = createRobotRectangle(pos);

        double[] x = xy[0];
        double[] y = xy[1];

        packet.fieldOverlay()
                .setStrokeWidth(strokeWidth)
                .setStroke(color)

                .strokePolygon(x, y);
    }

    public void createCircularRobot(Pose2D pos) {
        double x = pos.y;
        double y = pos.x;
        double h = pos.heading;

        double tX = x+(radius * Math.cos(h-Math.toRadians(90)));
        double tY = y+(radius * Math.sin(h-Math.toRadians(90)));

        packet.fieldOverlay()
                .setStrokeWidth(strokeWidth)
                .setStroke(color)
                .strokeCircle(x, -y, radius)
                .strokeLine(x, -y, tX, -tY);
    }




    public TelemetryPacket getPacket() {
        return packet;
    }
}
