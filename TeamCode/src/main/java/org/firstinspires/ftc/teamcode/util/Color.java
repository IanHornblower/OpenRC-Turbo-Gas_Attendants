package org.firstinspires.ftc.teamcode.util;

public class Color {

    /**
     * Converts from the HSB (hue, saturation, brightness) color model to the
     * RGB (red, green, blue) color model. The hue may be any floating point;
     * it's fractional portion is used to select the angle in the HSB model.
     * The saturation and brightness must be between 0 and 1. The result is
     * suitable for creating an RGB color with the one-argument constructor.
     *
     * @param hue        the hue of the HSB value
     * @param saturation the saturation of the HSB value
     * @param value the brightness of the HSB value
     * @return the RGB value
     */
    public static int[] HSVtoRGB(float hue, float saturation, float value) {
        if (saturation == 0)
            return convert(value, value, value);
        if (saturation < 0 || saturation > 1 || value < 0 || value > 1)
            throw new IllegalArgumentException();
        int h = (int) (hue * 6);
        float f = hue * 6 - h;
        float p = value * (1 - saturation);
        float q = value * (1 - f * saturation);
        float t = value * (1 - (1 - f) * saturation);

        switch (h) {
            case 0:
            case 6:
                return convert(value, t, p);
            case 1:
                return convert(q, value, p);
            case 2:
                return convert(p, value, t);
            case 3:
                return convert(p, q, value);
            case 4:
                return convert(t, p, value);
            case 5:
                return convert(value, p, q);
            default:
                throw new InternalError("impossible");
        }
    }


    private static int[] convert(float red, float green, float blue) {
        if (red < 0 || red > 1 || green < 0 || green > 1 || blue < 0 || blue > 1)
            throw new IllegalArgumentException("Bad RGB values");
        int redval = Math.round(255 * red);
        int greenval = Math.round(255 * green);
        int blueval = Math.round(255 * blue);
        if (redval > 255) redval = 255;
        if (greenval > 255) greenval = 255;
        if (blueval > 255) blueval = 255;
        int arr[] = {redval, greenval, blueval};
        return arr;
    }


}
