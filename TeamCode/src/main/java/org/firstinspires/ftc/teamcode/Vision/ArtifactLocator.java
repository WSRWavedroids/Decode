/*
 * Copyright (c) 2024 Phil Malone
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.Vision;

import static android.os.SystemClock.sleep;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.annotation.SuppressLint;
import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.CustomColorRange;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Teleop.SorterHardware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * This OpMode illustrates how to use a video source (camera) to locate specifically colored regions.
 * This sample is targeted towards circular blobList. To see rectangles, look at ConceptVisionColorLocator_Rectangle
 *
 * Unlike a "color sensor" which determines the color of nearby object, this "color locator"
 * will search the Region Of Interest (ROI) in a camera image, and find any "blobList" of color that
 * match the requested color range.  These blobList can be further filtered and sorted to find the one
 * most likely to be the item the user is looking for.
 *
 * To perform this function, a VisionPortal runs a ColorBlobLocatorProcessor process.
 *   The ColorBlobLocatorProcessor (CBLP) process is created first, and then the VisionPortal is built.
 *   The (CBLP) analyses the ROI and locates pixels that match the ColorRange to form a "mask".
 *   The matching pixels are then collected into contiguous "blobList" of pixels.
 *   The outer boundaries of these blobList are called its "contour".  For each blob, the process then
 *   creates the smallest possible circle that will fully encase the contour. The user can then call
 *   getBlobs() to retrieve the list of Blobs, where each contains the contour and the circle.
 *   Note: The default sort order for Blobs is ContourArea, in descending order, so the biggest
 *   contours are listed first.
 *
 * A colored enclosing circle is drawn on the camera preview to show the location of each Blob
 * The original Blob contour can also be added to the preview.
 * This is helpful when configuring the ColorBlobLocatorProcessor parameters.
 *
 * Tip:  Connect an HDMI monitor to the Control Hub to view the Color Location process in real-time.
 *       Or use a screen copy utility like ScrCpy.exe to view the video remotely.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

public class ArtifactLocator {

    private ExposureControl exposureControl;
    private GainControl gainControl;
    private WebcamName internalCamera;
    private Servo servo;
    private TouchSensor magnet;

    private ColorBlobLocatorProcessor purpleLocator;
    private ColorBlobLocatorProcessor greenLocator;
    private VisionPortal portal;
    private List<ColorBlobLocatorProcessor.Blob> purpleBlobList;
    private List<ColorBlobLocatorProcessor.Blob> greenBlobList;
    public enum slotState{EMPTY, PURPLE, GREEN}
    public enum positionState{FIRE, LOAD, SWITCH}

    public slot slotA;
    public slot slotB;
    public slot slotC;
    private slotRange zone1;
    private slotRange zone2;
    private slotRange zone3;
    private slotRange zone4;
    private slotRange zone5;
    private slotRange zone6;
    public ArrayList<slot> allSlots = new ArrayList<>();
    public ArrayList<slotRange> allZones = new ArrayList<slotRange>();
    public ArrayList<Double> offsetPositions = new ArrayList<>();
    public slotInventory inventory;

    public Robot robot;
    public ArtifactLocator(Robot robotFile) {
        robot = robotFile;
    }

    public void initCamera() {
        /* Build a "Color Locator" vision processor based on the ColorBlobLocatorProcessor class.
         * - Specify the color range you are looking for. Use a predefined color, or create your own
         *
         *   .setTargetColorRange(ColorRange.BLUE)     // use a predefined color match
         *     Available colors are: RED, BLUE, YELLOW, GREEN, ARTIFACT_GREEN, ARTIFACT_PURPLE
         *   .setTargetColorRange(new ColorRange(ColorSpace.YCrCb,  // or define your own color match
         *                                       new Scalar( 32, 176,  0),
         *                                       new Scalar(255, 255, 132)))
         *
         * - Focus the color locator by defining a RegionOfInterest (ROI) which you want to search.
         *     This can be the entire frame, or a sub-region defined using:
         *     1) standard image coordinates or 2) a normalized +/- 1.0 coordinate system.
         *     Use one form of the ImageRegion class to define the ROI.
         *       ImageRegion.entireFrame()
         *       ImageRegion.asImageCoordinates(50, 50,  150, 150)  100x100 pixels at upper left corner
         *       ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5)  50% width/height in center
         *
         * - Define which contours are included.
         *   You can get ALL the contours, ignore contours that are completely inside another contour.
         *     .setContourMode(ColorBlobLocatorProcessor.ContourMode.ALL_FLATTENED_HIERARCHY)
         *     .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
         *     EXTERNAL_ONLY helps to avoid bright reflection spots from breaking up solid colors.
         *
         * - Turn the displays of contours ON or OFF.
         *     Turning these on helps debugging but takes up valuable CPU time.
         *        .setDrawContours(true)                Draws an outline of each contour.
         *        .setEnclosingCircleColor(int color)   Draws a circle around each contour. 0 to disable.
         *        .setBoxFitColor(int color)            Draws a rectangle around each contour. 0 to disable. ON by default.
         *
         *
         * - include any pre-processing of the image or mask before looking for Blobs.
         *     There are some extra processing you can include to improve the formation of blobList.
         *     Using these features requires an understanding of how they may effect the final
         *     blobList.  The "pixels" argument sets the NxN kernel size.
         *        .setBlurSize(int pixels)
         *        Blurring an image helps to provide a smooth color transition between objects,
         *        and smoother contours.  The higher the number, the more blurred the image becomes.
         *        Note:  Even "pixels" values will be incremented to satisfy the "odd number" requirement.
         *        Blurring too much may hide smaller features.  A size of 5 is good for a 320x240 image.
         *
         *     .setErodeSize(int pixels)
         *        Erosion removes floating pixels and thin lines so that only substantive objects remain.
         *        Erosion can grow holes inside regions, and also shrink objects.
         *        "pixels" in the range of 2-4 are suitable for low res images.
         *
         *     .setDilateSize(int pixels)
         *        Dilation makes objects and lines more visible by filling in small holes, and making
         *        filled shapes appear larger. Dilation is useful for joining broken parts of an
         *        object, such as when removing noise from an image.
         *        "pixels" in the range of 2-4 are suitable for low res images.
         *
         *        .setMorphOperationType(MorphOperationType morphOperationType)
         *        This defines the order in which the Erode/Dilate actions are performed.
         *        OPENING:    Will Erode and then Dilate which will make small noise blobList go away
         *        CLOSING:    Will Dilate and then Erode which will tend to fill in any small holes in blob edges.
         */
        purpleLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(org.firstinspires.ftc.teamcode.CustomColorRange.ARTIFACT_PURPLE)   // Use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setBoxFitColor(Color.rgb(255, 0, 255))       // Disable the drawing of rectangles
                //.setCircleFitColor(Color.rgb(255, 0, 255)) // Draw a circle
                .setBlurSize(5)          // Smooth the transitions between different colors in image

                // the following options have been added to fill in perimeter holes.
                /*.setDilateSize(15)       // Expand blobList to fill any divots on the edges
                .setErodeSize(15)        // Shrink blobList back to original size
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)*/

                .build();

        greenLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(CustomColorRange.ARTIFACT_GREEN)   // Use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setBoxFitColor(Color.rgb(0, 255, 0))       // Disable the drawing of rectangles
                //.setCircleFitColor(Color.rgb(0, 255, 0)) // Draw a circle
                .setBlurSize(5)          // Smooth the transitions between different colors in image

                // the following options have been added to fill in perimeter holes.
                /*.setDilateSize(15)       // Expand blobList to fill any divots on the edges
                .setErodeSize(15)        // Shrink blobList back to original size
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)*/

                .build();

        internalCamera = hardwareMap.get(WebcamName.class, "CamCam");
        servo = hardwareMap.get(Servo.class, "blenderServo");
        magnet = hardwareMap.get(TouchSensor.class, "placeholder"); //TODO

        portal = new VisionPortal.Builder()
                .addProcessor(purpleLocator)
                .addProcessor(greenLocator)
                .setCameraResolution(new Size(320, 240))
                .setCamera(internalCamera)
                .build();

        //Define slots
        slotA = new slot(1);
        slotB = new slot(2);
        slotC = new slot(3);

        zone1 = new slotRange(0,0,0,0); //TODO fill in values
        zone2 = new slotRange(0,0,0,0); //TODO fill in values
        zone3 = new slotRange(0,0,0,0); //TODO fill in values
        zone4 = new slotRange(0,0,0,0); //TODO fill in values
        zone5 = new slotRange(0,0,0,0); //TODO fill in values
        zone6 = new slotRange(0,0,0,0); //TODO fill in values

        //Sort things into lists
        offsetPositions.add(slotA.servoLoadPosition); offsetPositions.add(slotB.servoFirePosition);
        offsetPositions.add(slotC.servoLoadPosition); offsetPositions.add(slotA.servoFirePosition);
        offsetPositions.add(slotB.servoLoadPosition); offsetPositions.add(slotC.servoFirePosition);

        allSlots.add(slotA); allSlots.add(slotB); allSlots.add(slotC);

        allZones.add(zone1); allZones.add(zone2); allZones.add(zone3);
        allZones.add(zone4); allZones.add(zone5); allZones.add(zone6);

        // Define the inventory
        inventory = new slotInventory();

        // Set camera settings
        /*while (portal.getCameraState() != VisionPortal.CameraState.CAMERA_DEVICE_READY) {
            sleep(10); //lol stallin
        }*/ sleep(2000); //TODO get this to actually work
        setCameraSettings();
    }

    public void setCameraSettings() {
        exposureControl = portal.getCameraControl(ExposureControl.class);
        exposureControl.setMode(ExposureControl.Mode.Manual);
        exposureControl.setExposure(37, TimeUnit.MILLISECONDS);

        gainControl = portal.getCameraControl(GainControl.class);
        gainControl.setGain(85);
    }

        // WARNING:  To view the stream preview on the Driver Station, this code runs in INIT mode.
        //while (opModeIsActive() || opModeInInit()) {

    public void update() {
        // Read the current list
        purpleBlobList = purpleLocator.getBlobs();
        greenBlobList = greenLocator.getBlobs();

        //FILTERS
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                50, 20000, purpleBlobList);  // filter out very small blobs.
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                50, 20000, greenBlobList);  // filter out very small blobs.

        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
                0.6, 1, purpleBlobList);     // filter out non-circular blobs.

        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
                0.6, 1, greenBlobList);     // filter out non-circular blobs.

        this.sortOutBlobs();
        this.takeInventory();

    }

    private void sortOutBlobs() {
        boolean z1Filled = false;
        boolean z2Filled = false;
        boolean z3Filled = false;
        boolean z4Filled = false;
        boolean z5Filled = false;
        boolean z6Filled = false;

        boolean inLoadPosition = getCurrentOffset() % 2 == 0;

        for (ColorBlobLocatorProcessor.Blob b : purpleBlobList) {
            Circle circleFit = b.getCircle();
            if (inLoadPosition & zone1.inRange(circleFit.getX(),circleFit.getY())) {
                findSlotByZone(zone1).occupied = slotState.PURPLE;
                z1Filled = true;
            } else if (!inLoadPosition & zone2.inRange(circleFit.getX(),circleFit.getY())) {
                findSlotByZone(zone2).occupied = slotState.PURPLE;
                z2Filled = true;
            } else if (inLoadPosition & zone3.inRange(circleFit.getX(),circleFit.getY())) {
                findSlotByZone(zone3).occupied = slotState.PURPLE;
                z3Filled = true;
            } else if (!inLoadPosition & zone4.inRange(circleFit.getX(),circleFit.getY())) {
                findSlotByZone(zone3).occupied = slotState.PURPLE;
                z4Filled = true;
            } else if (inLoadPosition & zone5.inRange(circleFit.getX(),circleFit.getY())) {
                findSlotByZone(zone3).occupied = slotState.PURPLE;
                z5Filled = true;
            } else if (!inLoadPosition & zone6.inRange(circleFit.getX(),circleFit.getY())) {
                findSlotByZone(zone3).occupied = slotState.PURPLE;
                z6Filled = true;
            }
        }

        for (ColorBlobLocatorProcessor.Blob b : greenBlobList) {
            Circle circleFit = b.getCircle();
            if (inLoadPosition & zone1.inRange(circleFit.getX(),circleFit.getY())) {
                findSlotByZone(zone1).occupied = slotState.GREEN;
                z1Filled = true;
            } else if (!inLoadPosition & zone2.inRange(circleFit.getX(),circleFit.getY())) {
                findSlotByZone(zone2).occupied = slotState.GREEN;
                z2Filled = true;
            } else if (inLoadPosition & zone3.inRange(circleFit.getX(),circleFit.getY())) {
                findSlotByZone(zone3).occupied = slotState.GREEN;
                z3Filled = true;
            } else if (!inLoadPosition & zone4.inRange(circleFit.getX(),circleFit.getY())) {
                findSlotByZone(zone3).occupied = slotState.GREEN;
                z4Filled = true;
            } else if (inLoadPosition & zone5.inRange(circleFit.getX(),circleFit.getY())) {
                findSlotByZone(zone3).occupied = slotState.GREEN;
                z5Filled = true;
            } else if (!inLoadPosition & zone6.inRange(circleFit.getX(),circleFit.getY())) {
                findSlotByZone(zone3).occupied = slotState.GREEN;
                z6Filled = true;
            }
        }

        for (slotRange r : allZones) {
            if (inLoadPosition & !z1Filled) {
                findSlotByZone(zone1).occupied = slotState.EMPTY;
            }
            if (!inLoadPosition & !z2Filled) {
                findSlotByZone(zone2).occupied = slotState.EMPTY;
            }
            if (inLoadPosition & !z3Filled) {
                findSlotByZone(zone3).occupied = slotState.EMPTY;
            }
            if (!inLoadPosition & !z4Filled) {
                findSlotByZone(zone3).occupied = slotState.EMPTY;
            }
            if (inLoadPosition & !z5Filled) {
                findSlotByZone(zone3).occupied = slotState.EMPTY;
            }
            if (!inLoadPosition & !z6Filled) {
                findSlotByZone(zone3).occupied = slotState.EMPTY;
            }
        }
    }

    private void takeInventory() {
        int currentPurpleCount = 0;
        int currentGreenCount = 0;
        for (slot currentSlot : allSlots) {
            switch (currentSlot.occupied) {
                case EMPTY:
                    break;
                case PURPLE:
                    currentPurpleCount ++;
                    break;
                case GREEN:
                    currentGreenCount ++;
                    break;
            }
        }
        inventory.count = currentPurpleCount + currentGreenCount;
        inventory.purpleCount = currentPurpleCount;
        inventory.greenCount = currentGreenCount;
        inventory.canMakePattern = inventory.purpleCount == 2 & inventory.greenCount == 1;
        return;
    }

    public slot findFirstType(slotState slotType) {
        for (slot currentSlot : allSlots) {
            if (currentSlot.occupied == slotType) {
                return currentSlot;
            }
        }
        return null;
    }

    public slot findSlotByZone(slotRange zone) {
        int offset = getCurrentOffset();
        if (offset == -1) {
            return null;
        }
        int zoneIndex = allZones.indexOf(zone) + 1;
        int x = zoneIndex - offset;

        switch (x) {
            case 1:
                return slotA;
            case 3:
                return slotB;
            case 5:
                return slotC;
        }
        return null;
    }

    /**
     * Will get the current offset, 0-5.
     * -1 means it's in transit.
     */
    public int getCurrentOffset() {
        if (SorterHardware.countToTarget(magnet, true)) {
            return offsetPositions.indexOf(servo.getPosition());
        } else {
            return -1;
        }
    }
    @SuppressLint("DefaultLocale")
    public void cameraTelemetry() {

        telemetry.addLine("Inventory: " + inventory.count + " Artifacts; " + inventory.purpleCount + " purple, " + inventory.greenCount + " green.");
        telemetry.addLine("Circularity Radius Center");
        telemetry.addLine("Gain: " + Integer.toString(gainControl.getGain()));


        // Display the Blob's circularity, and the size (radius) and center location of its circleFit.
        telemetry.addLine("Purple:");
        for (ColorBlobLocatorProcessor.Blob b : purpleBlobList) {

            Circle circleFit = b.getCircle();
            telemetry.addLine(String.format("%5.3f      %3d     (%3d,%3d)",
                    b.getCircularity(), (int) circleFit.getRadius(), (int) circleFit.getX(), (int) circleFit.getY()));
        }

        telemetry.addLine("Green");
        for (ColorBlobLocatorProcessor.Blob b : greenBlobList) {

            Circle circleFit = b.getCircle();
            telemetry.addLine(String.format("%5.3f      %3d     (%3d,%3d)",
                    b.getCircularity(), (int) circleFit.getRadius(), (int) circleFit.getX(), (int) circleFit.getY()));
        }
    }

    /**
     * The slot class stores the occupancy state (slotState enum), slotIdentity (effectively starting position),
     * and correct servo positions to load and fire an Artifact. A constructor must be called to create an
     * instance of the class, representing one slot in the blender.
     */
    public class slot {
        public slotState occupied;
        public int slotIdentity;
        public double servoFirePosition;
        public double servoLoadPosition;
        public slot(int slotIdentity) {
            this.slotIdentity = slotIdentity;
            switch (this.slotIdentity) {
                case 1: servoFirePosition = SorterHardware.slotALaunch; servoLoadPosition = SorterHardware.slotAIntake; break;
                case 2: servoFirePosition = SorterHardware.slotBLaunch; servoLoadPosition = SorterHardware.slotBIntake; break;
                case 3: servoFirePosition = SorterHardware.slotCLaunch; servoLoadPosition = SorterHardware.slotCIntake; break;
                default: throw new IllegalArgumentException("Invalid slotIdentity"); // Fancy code throw error if bad
            }
        }
    }

    public class slotInventory {
        public int count;
        public int purpleCount;
        public int greenCount;
        public boolean canMakePattern;
        public slotInventory() {}
    }

    /**
     * The slotRange class is used to define and range of values for the camera to use. If the center of a Blob
     * is within the range (which is easy to check by calling the inRange() boolean function), the Artifact
     * represented by the Blob is within the slot for which the range represents.
     */
    public class slotRange {
        private final float xMin;
        private final float xMax;
        private final float yMin;
        private final float yMax;
        slotRange(float xMin, float xMax, float yMin, float yMax) {
            this.xMin = xMin;
            this.xMax = xMax;
            this.yMin = yMin;
            this.yMax = yMax;
        }

        /**
         * The inRange() function checks to see if a given coordinate is within the slotRange. Its intended
         * implementation is to check whether or not an Artifact, represented by a Blob, is within a slot,
         * represented by a slotRange. Input the center coordinates of a Blob to test this.
         * @param inputX
         * @param inputY
         * @return Whether or not the point is in the range (Boolean)
         */
        public boolean inRange(float inputX, float inputY) {
            return inputX > xMin & inputX < xMax & inputY > yMin & inputY < yMax;
        }
    }
}