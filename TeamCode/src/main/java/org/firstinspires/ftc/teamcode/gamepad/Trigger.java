package org.firstinspires.ftc.teamcode.gamepad;

public class Trigger {

    enum SIDE {
        left,
        right
    }

    public static enum PRESS {
        lightly,
        halfway,
        down,
        unpressed
    }

    double light = 0.1;
    double half = 0.4;
    double down = 0.7;

    boolean isPressed = false;
    double value = 0;
    PRESS press = PRESS.unpressed;

    GamepadEx gamepadEx;
    SIDE side;

    public Trigger(GamepadEx gamepadEx, SIDE side) {
        this.gamepadEx = gamepadEx;
        this.side = side;
    }

    public boolean isPressed() {
        switch (side) {
            case right:
                isPressed = gamepadEx.gamepad().right_trigger > 0.1;
                break;
            case left:
                isPressed = gamepadEx.gamepad().left_trigger > 0.1;
                break;
        }
        return isPressed;
    }

    public double value() {
        switch (side) {
            case right:
                value = gamepadEx.gamepad().right_trigger;
                break;
            case left:
                value = gamepadEx.gamepad().left_trigger;
                break;
        }
        return value;
    }

    public PRESS howPressed() {
        switch (side) {
            case right:
                if(gamepadEx.gamepad().right_trigger > light && gamepadEx.gamepad().right_trigger < half) press = PRESS.lightly;
                else if(gamepadEx.gamepad().right_trigger > half && gamepadEx.gamepad().right_trigger < down) press = PRESS.halfway;
                else if(gamepadEx.gamepad().right_trigger > down) press = PRESS.down;
                else press = PRESS.unpressed;
                break;
            case left:
                if(gamepadEx.gamepad().left_trigger > light && gamepadEx.gamepad().left_trigger < half) press = PRESS.lightly;
                else if(gamepadEx.gamepad().left_trigger > half && gamepadEx.gamepad().left_trigger < down) press = PRESS.halfway;
                else if(gamepadEx.gamepad().left_trigger > down) press = PRESS.down;
                else press = PRESS.unpressed;
                break;
        }
        return press;
    }
}
