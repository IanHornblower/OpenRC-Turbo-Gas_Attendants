package org.firstinspires.ftc.teamcode.hardware;

import android.util.Log;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.internal.ui.GamepadUser;


import java.util.Map;

public class Controller {

    public static final int ID_UNASSOCIATED = -1;
    public static final int ID_SYNTHETIC = -2;

    protected long time = System.currentTimeMillis();
    protected long cycleLength = 0;
    protected long ttime = 0;

    public Gamepad gamepad;

    public Controller(Gamepad gamepad) {
        this.gamepad = gamepad;
    }
    /**
     * Updates the Gamepad
     */
    public void update() {
        updateInputs();
        cycleLength = System.currentTimeMillis() - time;
        time = System.currentTimeMillis();
        ttime = ttime + cycleLength;

        // Set Touch Positions to 0 when you let go
        if (!gamepad.touchpad_finger_1) {
            gamepad.touchpad_finger_1_x = 0;
            gamepad.touchpad_finger_1_y = 0;
        }
        if (!gamepad.touchpad_finger_2) {
            gamepad.touchpad_finger_2_x = 0;
            gamepad.touchpad_finger_2_y = 0;
        }

        // Set Touch Region Bools
        touchingTop = gamepad.touchpad_finger_1_y > 0;
        touchingBottom = gamepad.touchpad_finger_1_y < 0;
        touchingLeft = gamepad.touchpad_finger_1_x > 0;
        touchingRight = gamepad.touchpad_finger_1_x < 0;
        touchingTopLeft = touchingTop && touchingLeft;
        touchingTopRight = touchingTop && touchingRight;
        touchingBottomLeft = touchingBottom && touchingLeft;
        touchingBottomRight = touchingBottom && touchingRight;

        Log.i("Controller.update()", "Controller Updated");

        if(gamepad.touchpad_finger_1 && ttime > 250) {
            loc3 = new Float[]{loc2[0], loc2[1]};
            loc2 = new Float[]{loc1[0], loc1[1]};
            loc1 = new Float[]{gamepad.touchpad_finger_1_x, gamepad.touchpad_finger_1_y};
            ttime = 0;
        }

    }


    public GamepadUser getUser() {
        return gamepad.getUser();
    }
    public Gamepad.Type type() {
        return gamepad.type();
    }
    public void rumble(int duration) {
        gamepad.rumble(duration);
    }

    public int id = ID_UNASSOCIATED;
    public boolean a = false;
    public boolean b = false;
    public boolean x = false;
    public boolean y = false;
    public boolean cross = false;
    public boolean triangle = false;
    public boolean square = false;
    public boolean circle = false;
    public boolean left_bumper = false;
    public boolean right_bumper = false;
    public boolean ps = false;
    public boolean start = false;
    public boolean select = false;
    public boolean options = false;
    public boolean share = false;
    public float left_stick_x = 0.0f;
    public float left_stick_y = 0.0f;
    public float right_stick_x = 0.0f;
    public float right_stick_y = 0.0f;
    public boolean dpad_up = false;
    public boolean dpad_down = false;
    public boolean dpad_left = false;
    public boolean dpad_right = false;
    public boolean touchpad_finger_1 = false;
    public boolean touchpad_finger_2 = false;
    public float touchpad_finger_1_x = 0.0f;
    public float touchpad_finger_1_y = 0.0f;
    public float touchpad_finger_2_x = 0.0f;
    public float touchpad_finger_2_y = 0.0f;



    public boolean touchingTop = false;
    public boolean touchingBottom = false;
    public boolean touchingLeft = false;
    public boolean touchingRight = false;
    public boolean touchingTopLeft = false;
    public boolean touchingTopRight = false;
    public boolean touchingBottomLeft = false;
    public boolean touchingBottomRight = false;
    public float right_trigger = 0.0f;
    public float left_trigger = 0.0f;


    public Float[] loc1 = {0f, 0f};
    public Float[] loc2 = {0f, 0f};
    public Float[] loc3 = {0f, 0f};

    private void updateInputs() {
        a = gamepad.a;
        b = gamepad.b;
        x = gamepad.x;
        y = gamepad.y;
        cross = gamepad.cross;
        square = gamepad.square;
        triangle = gamepad.triangle;
        circle = gamepad.circle;
        left_bumper = gamepad.left_bumper;
        right_bumper = gamepad.right_bumper;
        ps = gamepad.ps;
        options = gamepad.options;
        share = gamepad.share;
        options = gamepad.options;
        share = gamepad.share;
        touchpad_finger_1 = gamepad.touchpad_finger_1;
        touchpad_finger_2 = gamepad.touchpad_finger_2;
        touchpad_finger_1_x = gamepad.touchpad_finger_1_x;
        touchpad_finger_1_y = gamepad.touchpad_finger_1_y;
        touchpad_finger_2_x = gamepad.touchpad_finger_2_x;
        touchpad_finger_2_y = gamepad.touchpad_finger_2_y;
        left_trigger = gamepad.left_trigger;
        right_trigger = gamepad.right_trigger;
    }

}
