package org.firstinspires.ftc.teamcode.gamepad;

public class Joystick {

    double x = 0, y = 0;
    double xMultiplier = 1;
    double yMultiplier = 1;

    enum SIDE {
        left,
        right
    }

    public static enum AXIS {
        x,
        y
    }

    GamepadEx gamepadEx;
    SIDE side;

    public Joystick(GamepadEx gamepadEx, SIDE side) {
        this.gamepadEx = gamepadEx;
        this.side = side;
    }

    public void invert(AXIS axis) {
        switch (axis) {
            case x:
                xMultiplier = -1;
                break;
            case y:
                yMultiplier = -1;
                break;
        }
    }

    public double x() {
        switch (side) {
            case left:
                x = gamepadEx.gamepad().left_stick_x * xMultiplier;
                break;
            case right:
                x = gamepadEx.gamepad().right_stick_x * xMultiplier;
                break;
        }
        return x;
    }

    public double y() {
        switch (side) {
            case left:
                y = gamepadEx.gamepad().left_stick_y * yMultiplier;
                break;
            case right:
                y = gamepadEx.gamepad().right_stick_y * yMultiplier;
                break;
        }
        return y;
    }

    public boolean isPressed() {
        return gamepadEx.gamepad().left_stick_button;
    }
}
