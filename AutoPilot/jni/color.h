/**
 * Color tools.
 *
 * @version Team Safier
 * @version 1.0
 */

/*
 * Type for color conversion.
 */
typedef struct {
    double h;       // angle in degrees
    double s;       // a fraction between 0 and 1
    double v;       // a fraction between 0 and 1
} hsv;

/*
 * Taken from Java's own implementation (translated to C).
 */
static hsv rgb_to_hsv(int r, int g, int b);
hsv rgb_to_hsv(int r, int g, int b) {
    
    hsv out;
    
    float hue, saturation, brightness;

    int cmax = (r > g) ? r : g;
    if (b > cmax)
        cmax = b;
    int cmin = (r < g) ? r : g;
    if (b < cmin)
        cmin = b;
    
    brightness = ((float) cmax) / 255.0f;
    if (cmax != 0)
        saturation = ((float) (cmax - cmin)) / ((float) cmax);
    else
        saturation = 0;
    if (saturation == 0)
        hue = 0;
    else {
        float redc = ((float) (cmax - r)) / ((float) (cmax - cmin));
        float greenc = ((float) (cmax - g)) / ((float) (cmax - cmin));
        float bluec = ((float) (cmax - b)) / ((float) (cmax - cmin));
        if (r == cmax)
            hue = bluec - greenc;
        else if (g == cmax)
            hue = 2.0f + redc - bluec;
        else
            hue = 4.0f + greenc - redc;
        hue = hue / 6.0f;
        if (hue < 0)
            hue = hue + 1.0f;
    }
    
    out.h = hue;
    out.s = saturation;
    out.v = brightness;
    return out;
    
}
