package org.firstinspires.ftc.teamcode.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadEx {

    private final Gamepad gamepad;
    public Joystick leftJoystick;
    public Joystick rightJoystick;
    public Trigger leftTrigger;
    public Trigger rightTrigger;

    public static enum DPAD {
        left,
        right,
        up,
        down,
        unpressed
    }

    public static enum BUMPER {
        left,
        right,
        both,
        unpressed
    }

    public GamepadEx(Gamepad gamepad) {
        this.gamepad = gamepad;
        this.leftJoystick = new Joystick(this, Joystick.SIDE.left);
        this.rightJoystick = new Joystick(this, Joystick.SIDE.right);
        this.leftTrigger = new Trigger(this, Trigger.SIDE.left);
        this.rightTrigger = new Trigger(this, Trigger.SIDE.right);
    }

    public Gamepad gamepad() {
        return gamepad;
    }

    /*
          // Bumpers //
     */

    public boolean cross() {
        return gamepad.cross;
    }

    public boolean circle() {
        return gamepad.circle;
    }

    public boolean square() {
        return gamepad.square;
    }

    public boolean triangle() {
        return gamepad.triangle;
    }

    /*
            // Dpad //
     */

    public boolean dpad_left() {
        return gamepad.dpad_left;
    }

    public boolean dpad_right() {
        return gamepad.dpad_right;
    }

    public boolean dpad_up() {
        return gamepad.dpad_up;
    }

    public boolean dpad_down() {
        return gamepad.dpad_down;
    }

    public DPAD dpad() {
        if(dpad_up()) return DPAD.up;
        else if(dpad_down()) return DPAD.down;
        else if(dpad_left()) return DPAD.left;
        else if(dpad_right()) return DPAD.right;
        else return DPAD.unpressed;
    }

    /*
            // Bumpers //
     */

    public boolean left_bumper() {
        return gamepad.left_bumper;
    }

    public boolean right_bumper() {
        return gamepad.right_bumper;
    }

    public BUMPER bumpers() {
        if(left_bumper()) return BUMPER.left;
        else if(right_bumper()) return BUMPER.right;
        else if(right_bumper() && left_bumper()) return BUMPER.both;
        else return BUMPER.unpressed;
    }

    /*
            // Rumble //
     */

    public void rumble(int duration) {
        gamepad.rumble(duration);
    }

    public void rumbleBlips(int count) {
        gamepad.rumbleBlips(count);
    }

}
