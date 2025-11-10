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
import static org.firstinspires.ftc.teamcode.Core.ArtifactLocator.slotState.EMPTY;
import static org.firstinspires.ftc.teamcode.Core.ArtifactLocator.slotState.PURPLE;
import static org.firstinspires.ftc.teamcode.Core.ArtifactLocator.slotState.GREEN;
import static org.firstinspires.ftc.teamcode.Core.ArtifactLocator.slotState.UNKNOWN;


import android.annotation.SuppressLint;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class ArtifactLocator {

    private ExposureControl exposureControl;
    private GainControl gainControl;

    private ColorBlobLocatorProcessor purpleLocator;
    private ColorBlobLocatorProcessor greenLocator;
    private VisionPortal portal;
    private List<ColorBlobLocatorProcessor.Blob> purpleBlobList;
    private List<ColorBlobLocatorProcessor.Blob> greenBlobList;
    public enum slotState{EMPTY, PURPLE, GREEN, UNKNOWN}


    public slot slotA;
    public slot slotB;
    public slot slotC;
    public slot noSlot;
    private slotRange zone1;
    private slotRange zone2;
    private slotRange zone3;
    private slotRange zone4;
    private slotRange zone5;
    private slotRange zone6;
    public ArrayList<slot> allSlots = new ArrayList<>();
    public ArrayList<slotRange> allZones = new ArrayList<slotRange>();
    public ArrayList<Integer> offsetPositions = new ArrayList<>();
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

/*        purpleLocator = new ColorBlobLocatorProcessor.Builder()
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

/*                .build();

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

/*                .build();

        portal = new VisionPortal.Builder()
                .addProcessor(purpleLocator)
                .addProcessor(greenLocator)
                .setCameraResolution(new Size(320, 240))
                .setCamera(robot.)
                .build();
*/

        //Define slots
        slotA = new slot(robot.sorterHardware.positions[0],robot.sorterHardware.positions[1]);
        slotB = new slot(robot.sorterHardware.positions[2],robot.sorterHardware.positions[3]);
        slotC = new slot(robot.sorterHardware.positions[4],robot.sorterHardware.positions[5]);
        noSlot = new slot();

        zone1 = new slotRange(140, 180, 0, 120);
        zone2 = new slotRange(0,160,0,120);
        zone3 = new slotRange(0,160,120,240);
        zone4 = new slotRange(140,180,120,240);
        zone5 = new slotRange(160,320,120,240);
        zone6 = new slotRange(160,320,0,120);

        //Sort things into lists
        offsetPositions.add(0, slotA.loadPosition);
        offsetPositions.add(1, slotB.firePosition);
        offsetPositions.add(2, slotC.loadPosition);
        offsetPositions.add(3, slotA.firePosition);
        offsetPositions.add(4, slotB.loadPosition);
        offsetPositions.add(5, slotC.firePosition);

        allSlots.add(slotA); allSlots.add(slotB); allSlots.add(slotC);

        allZones.add(zone1); allZones.add(zone2); allZones.add(zone3);
        allZones.add(zone4); allZones.add(zone5); allZones.add(zone6);

        // Define the inventory
        inventory = new slotInventory();

        // Set camera settings
        /*while (portal.getCameraState() != VisionPortal.CameraState.CAMERA_DEVICE_READY) {
            sleep(10); //lol stallin
        }*/ sleep(2000); //TODO get this to actually work
        //setCameraSettings();
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
     * Queries the camera for the current blob lists, sorts them into slots, and updates the
     * inventory class.
     */
    public void update() {
        // Read the current list
        //purpleBlobList = purpleLocator.getBlobs();
        //greenBlobList = greenLocator.getBlobs();

        //FILTERS
        /*ColorBlobLocatorProcessor.Util.filterByCriteria(
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
                0.6, 1, greenBlobList);     // filter out non-circular blobs.*/

        this.takeInventory();

    }

    /**
     * Sorts the color blobs into the proper slots.
     */
    public void sortOutBlobs(slotRange currentZone, int state) {
        slotState newState;

        switch (state) {
            case 0: newState = EMPTY;
            case 1: newState = PURPLE;
            case 2: newState = GREEN;
            default: newState = EMPTY;
        }

        this.findSlotByZone(currentZone).setOccupied(newState);
    }

    /**
     * Totals the number of Artifacts stored in the blender and updates the inventory class.
     */
    private void takeInventory() {
        int currentPurpleCount = 0;
        int currentGreenCount = 0;
        for (slot currentSlot : allSlots) {
            switch (currentSlot.getOccupied()) {
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
     * @param slotType The target state of the slot. Can be EMPTY, GREEN, PURPLE, or UNKNOWN
     *                 in the form of a slotState enum.
     * @return The first found slot filled with the indicated slotType.
     */
    public slot findFirstType(slotState slotType) {
        for (slot currentSlot : allSlots) {
            if (currentSlot.contains(slotType)) {
                return currentSlot;
            }
        }
        return noSlot;
    }

    /**
     * Searches the slots in order of ABC to find the first slot that is known and does not contain
     * the specified contents. If the input is UNKNOWN, it will find the first known slot.
     * @param slotType The not-target state of the slot. Can be EMPTY, GREEN, PURPLE, or UNKNOWN in
     *                 the form of a slotState enum.
     * @return The first found slot.
     */
    public slot findFirstNotType(slotState slotType) {
        for (slot currentSlot : allSlots) {
            if (currentSlot.doesNotContain(slotType, UNKNOWN)) {
                return currentSlot;
            }
        }
        return noSlot;
    }

    /**
     * Uses the current offset to find which slot is in the specified zone.
     * @param zone The target zone.
     * @return The found slot.
     */
    public slot findSlotByZone(slotRange zone) {
        int offset = getCurrentOffset();
        if (offset == -1) {
            return noSlot;
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
        return noSlot;
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
     * @return The current nearest blender offset, from 0-5. -2 = error.
     */
    private int findClosestOffset(double ticks) {
        ticks = equalizeMotorPositions((int) ticks);

        double lowestDistance = 1000000000;
        int offset = -2;

        for(int i = 0; i<6; i++) {
            double currentDistanceCheck = Math.abs(offsetPositions.get(i) - ticks);
            if (currentDistanceCheck < lowestDistance) {
                lowestDistance = currentDistanceCheck;
                offset = i;
            }
        }

        return offset;
    }

    /**
     * Does some basic math to equalize a motor position between 0 and SorterHardware.ticksPerRotation (8192).
     * @param ticks The number to be equalized.
     * @return The equalized position, between 0-8192.
     */
    public int equalizeMotorPositions(int ticks) {
        while (ticks > robot.sorterHardware.ticksPerRotation) {
            ticks -= robot.sorterHardware.ticksPerRotation;
        }
        while (ticks < 0) {
            ticks += robot.sorterHardware.ticksPerRotation;
        }
        return ticks;
    }

    /**
     * Adds the currently detected blobs to telemetry. Will not update telemetry.
     */
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
     * The slot class stores the occupancy state (slotState enum) and correct motor
     * positions to load and fire an Artifact. A constructor must be called to create an
     * instance of the class, representing one slot in the blender.
     */
    public class slot {
        private slotState occupied = UNKNOWN;
        private int firePosition;
        private int loadPosition;
        private boolean specialIsNotASlot = false;

        /**
         * This constructor creates a slot.
         * @param motorLoadPosition The motor position to go to when firing, in ticks.
         * @param motorFirePosition The motor position to go to when loading, in ticks.
         */
        public slot(double motorLoadPosition, double motorFirePosition) {
            this.loadPosition = (int) motorLoadPosition;
            this.firePosition = (int) motorFirePosition;
        }

        /**
         * This constructor is a special case. It will create a "fake" slot that always returns the
         * current position. This is to ensure the findFirstSlot(), findFirstNoSlot(), and
         * findSlotByZone() functions can still return a "no slot found" option without returning
         * null. Note that noSlot technically could end up with junk data in slotState occupied.
         */
        public slot() {
            this.specialIsNotASlot = true;
        }

        /**
         * Returns the stored firing position for the slot. If the slot is noSlot,
         * will return the current motor position, effectively not moving.
         * @return The firing position, in ticks
         */
        public int getFirePosition() {
            if(specialIsNotASlot) {
                return robot.sorterHardware.motor.getCurrentPosition();
            }
            else return firePosition;
        }

        /**
         * Returns the stored loading position for the slot. If the slot is noSlot,
         * will return the current motor position.
         * @return The loading position, in ticks
         */
        public int getLoadPosition() {
            if(specialIsNotASlot) {
                return robot.sorterHardware.motor.getCurrentPosition();
            }
            return loadPosition;
        }

        /**
         * Returns the current occupation of the slot. if the slot is noSlot, will return UNKNOWN.
         * @return The current occupation of the slot, in a slotState enum.
         */
        public slotState getOccupied() {
            if (specialIsNotASlot) {
                return UNKNOWN;
            }
            return occupied;
        }

        /**
         * Stores the new contents in the slot.
         * @param newOccupation The new slotState
         */
        public void setOccupied(slotState newOccupation) {
            occupied = newOccupation;
        }

        /**
         * Checks to see if the slot contains the specified contents
         * @param checkState The contents to check for, in the form iof a slotState enum.
         * @return Whether or not the slot contains the contents checked for.
         */
        public boolean contains(slotState checkState) {
            return occupied == checkState;
        }

        /**
         * Checks to see if the slot doesn't contain any of the specified contents.
         * @param checkStates The state(s) to be checked against. Yes, state(s); it can handle multiple.
         * @return Whether or not the slot does not contain any of the contents checked against.
         */
        public boolean doesNotContain(slotState... checkStates) {
            for (slotState currentCheckState : checkStates) {
                if (currentCheckState == occupied) {
                    return false;
                }
            }
            return true;
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
     * The slotRange class is used to define and range of values for the camera to use. If the
     * center of a Blob is within the range (which is easy to check by calling the inRange() boolean
     * function), the Artifact represented by the Blob is within the slot for which the range
     * represents.
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
         * The inRange() function checks to see if a given coordinate is within the slotRange. Its
         * intended implementation is to check whether or not an Artifact, represented by a Blob, is
         * within a slot, represented by a slotRange. Input the center coordinates of a Blob to test
         * this.
         * @param inputX The x-coordinate to test
         * @param inputY The y-coordinate to test
         * @return Whether or not the point is in the range (Boolean)
         */
        public boolean inRange(float inputX, float inputY) {
            return inputX > xMin & inputX < xMax & inputY > yMin & inputY < yMax;
        }
    }
}