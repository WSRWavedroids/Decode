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

package org.firstinspires.ftc.teamcode.Core;

import static android.os.SystemClock.sleep;

import android.annotation.SuppressLint;
import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.Vision.CustomColorRange;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

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
        initCamera();
    }

    /**
     * Initializes the camera, sets the settings, prepares the VisionPortal,
     * and initializes numerous logic-related objects, including slots and the inventory.
     * Might take a couple seconds to run, as it has to wait for the camera to initialize.
     */
    public void initCamera() {

        purpleLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(CustomColorRange.ARTIFACT_PURPLE)   // Use a predefined color match
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

        // Hardware Map
        internalCamera = robot.hardwareMap.get(WebcamName.class, "CamCam");
        magnet = robot.hardwareMap.get(TouchSensor.class, "placeholder"); //TODO

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
        offsetPositions.add(slotA.motorLoadPosition); offsetPositions.add(slotB.motorFirePosition);
        offsetPositions.add(slotC.motorLoadPosition); offsetPositions.add(slotA.motorFirePosition);
        offsetPositions.add(slotB.motorLoadPosition); offsetPositions.add(slotC.motorFirePosition);

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

    /**
     * Sets the proper camera settings. Must be called after the camera is initialized,
     * otherwise the program crashes.
     */
    public void setCameraSettings() {
        exposureControl = portal.getCameraControl(ExposureControl.class);
        exposureControl.setMode(ExposureControl.Mode.Manual);
        exposureControl.setExposure(37, TimeUnit.MILLISECONDS);

        gainControl = portal.getCameraControl(GainControl.class);
        gainControl.setGain(85);
    }

    /**
     * Queries the camera for the current blob lists, sorts them into slots, and updates the inventory class.
     */
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

    /**
     * Sorts the color blobs into the proper slots.
     */
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

    /**
     * Totals the number of Artifacts stored in the blender and updates the inventory class.
     */
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

    /**
     * Searches the slots in order of ABC to find the first slot containing the indicated contents.
     * @param slotType The target state of the slot. Can be EMPTY, GREEN, or PURPLE,
     *                 in the form of a slotState enum.
     * @return The first found slot filled with the indicated slotType.
     */
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

    //TODO Move to LauncherHardware???
    /**
     * Uses the blender encoder and offsetPositions to calculate which
     * offset position the blender is currently in.
     * @return The current offset, 0-5. -1 means it's in transit.
     */
    public int getCurrentOffset() {
        if (robot.sorterHardware.positionedCheck()) {
            return findClosestOffset(robot.sorterHardware.motor.getCurrentPosition());
        } else {
            return -1;
        }
    }

    //TODO Move to LauncherHardware??? Maybe replace it with magnets.
    /**
     * Compares the current motor position to offsetPositions to
     * calculate which offset position the blender is closest to.
     * @param ticks The current position of the encoder
     * @return The current nearest blender offset, from 0-5. -1 = error.
     */
    private int findClosestOffset(double ticks) {
        while (ticks > robot.sorterHardware.ticksPerRotation) {
            ticks -= robot.sorterHardware.ticksPerRotation;
        }
        while (ticks < robot.sorterHardware.ticksPerRotation) {
            ticks += robot.sorterHardware.ticksPerRotation;
        }

        double lowestDistance = 1000000000;
        int offset = -1;

        for(int i = 0; i<6; i++) {
            double currentDistanceCheck = Math.abs(offsetPositions.get(i) - ticks);
            if (currentDistanceCheck < lowestDistance) {
                lowestDistance = currentDistanceCheck;
                offset = i;
            }
        }

        return offset;
    }

    @SuppressLint("DefaultLocale")
    public void cameraTelemetry() {

        robot.telemetry.addLine("Inventory: " + inventory.count + " Artifacts; " + inventory.purpleCount + " purple, " + inventory.greenCount + " green.");
        robot.telemetry.addLine("Circularity Radius Center");
        robot.telemetry.addLine("Gain: " + Integer.toString(gainControl.getGain()));


        // Display the Blob's circularity, and the size (radius) and center location of its circleFit.
        robot.telemetry.addLine("Purple:");
        for (ColorBlobLocatorProcessor.Blob b : purpleBlobList) {

            Circle circleFit = b.getCircle();
            robot.telemetry.addLine(String.format("%5.3f      %3d     (%3d,%3d)",
                    b.getCircularity(), (int) circleFit.getRadius(), (int) circleFit.getX(), (int) circleFit.getY()));
        }

        robot.telemetry.addLine("Green");
        for (ColorBlobLocatorProcessor.Blob b : greenBlobList) {

            Circle circleFit = b.getCircle();
            robot.telemetry.addLine(String.format("%5.3f      %3d     (%3d,%3d)",
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
        public double motorFirePosition;
        public double motorLoadPosition;
        public slot(int slotIdentity) {
            this.slotIdentity = slotIdentity;
            switch (this.slotIdentity) {
                case 1: motorFirePosition = robot.sorterHardware.positions[1]; motorLoadPosition = robot.sorterHardware.positions[0]; break;
                case 2: motorFirePosition = robot.sorterHardware.positions[3]; motorLoadPosition = robot.sorterHardware.positions[2]; break;
                case 3: motorFirePosition = robot.sorterHardware.positions[5]; motorLoadPosition = robot.sorterHardware.positions[4]; break;
                default: throw new IllegalArgumentException("Invalid slotIdentity"); // Fancy code throw error if bad
            }
        }
    }

    /**
     * This class is the inventory. It stores the current count of Artifacts,
     * for each color and in total, and whether or not a Pattern can be created.
     */
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