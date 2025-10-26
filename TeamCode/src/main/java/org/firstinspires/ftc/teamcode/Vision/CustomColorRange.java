package org.firstinspires.ftc.teamcode.Vision;

import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.opencv.core.Scalar;

public class CustomColorRange extends ColorRange {
    protected final ColorSpace colorSpace;
    protected final Scalar min;
    protected final Scalar max;

    /**
     * This is the ColorRange for a purple Artifact
     */
    public static final ColorRange ARTIFACT_PURPLE = new ColorRange(
            ColorSpace.HSV,
            new Scalar(134,  70,  70),
            new Scalar(154, 255, 255)
    );

    /**
     * This is the ColorRange for a green Artifact
     */
    public static final ColorRange ARTIFACT_GREEN = new ColorRange(
            ColorSpace.HSV,
            new Scalar(55,  50,  70),
            new Scalar(85, 255, 255)
    );

    /**
     * This ColorRange should encompass the full range of everything the camera can see.
     */
    public static final ColorRange EVERYTHING = new ColorRange(
            ColorSpace.RGB,
            new Scalar (0,0,0),
            new Scalar (255,255,255)
    );

    /**
     * Allows you to create your own ColorRange if you need to.
     * @param colorSpace
     * @param min
     * @param max
     */
    public CustomColorRange(ColorSpace colorSpace, Scalar min, Scalar max) {
        super(colorSpace,min,max);
        this.colorSpace = colorSpace;
        this.min = min;
        this.max = max;
    }
}