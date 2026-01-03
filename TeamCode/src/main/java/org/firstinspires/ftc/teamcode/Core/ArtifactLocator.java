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

import static org.firstinspires.ftc.teamcode.Core.ArtifactLocator.SlotState.*;
import static org.firstinspires.ftc.teamcode.Core.BetaSorterHardware.positionState.*;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.ml.EM;

import java.util.ArrayList;

public class ArtifactLocator {

    //private ExposureControl exposureControl;
    //private GainControl gainControl;

    //private ColorBlobLocatorProcessor purpleLocator;
    //private ColorBlobLocatorProcessor greenLocator;
    //private VisionPortal portal;
    //private List<ColorBlobLocatorProcessor.Blob> purpleBlobList;
    //private List<ColorBlobLocatorProcessor.Blob> greenBlobList;
    public enum SlotState {EMPTY, PURPLE, GREEN, UNKNOWN}

    public Slot slotA;
    public Slot slotB;
    public Slot slotC;
    public Slot noSlot;
    public Zone zone1;
    public Zone zone2;
    public Zone zone3;
    public ArrayList<Slot> allSlots = new ArrayList<>();
    public ArrayList<Zone> allZones = new ArrayList<>();
    public ArrayList<Integer> offsetPositions = new ArrayList<>();
    public SlotInventory inventory;

    public Robot robot;

    private ElapsedTime sortCooldown = new ElapsedTime();

    public ArtifactLocator(Robot robotFile) {
        robot = robotFile;
        initLogic();
    }

    /**
     * Initializes numerous logic-related objects, including slots, the inventory, and lists to
     * iterate through them.
     */
    public void initLogic() {
        double ticksPerRotation = BetaSorterHardware.ticksPerRotation;
        // Define slots
        slotA = new Slot(0, (int) (ticksPerRotation / 3), "A");
        slotB = new Slot((int) (2 * ticksPerRotation / 3), 0, "B");
        slotC = new Slot((int) (ticksPerRotation / 3), (int) (2 * ticksPerRotation / 3), "C");
        noSlot = new NoSlot();

        zone1 = new Zone(140, 180, 0, 120);
        zone2 = new Zone(0, 160, 0, 120);
        zone3 = new Zone(0, 160, 120, 240);

        //Sort things into lists
        offsetPositions.add(0, slotA.getLoadPosition());
        offsetPositions.add(1, slotC.getLoadPosition());
        offsetPositions.add(2, slotB.getLoadPosition());

        allSlots.add(slotA); allSlots.add(slotB); allSlots.add(slotC);

        allZones.add(zone1); allZones.add(zone2); allZones.add(zone3);

        // Define the inventory
        inventory = new SlotInventory();
    }

    /**
     * Queries the camera for the current blob lists, sorts them into slots, and updates the
     * inventory class.
     */
    public void update() {
        // Read the current list
        this.takeInventory();
    }

    /**
     * Sorts the HushyLens output into the appropriate Slot.
     * @param state The HuskyLens ID. 1 = PURPLE, 2 = GREEN, 3 = EMPTY.
     */
    public void sortOutBlobs(int state) {
        SlotState newState;
        Slot loadSlot;

        switch (state) {
            case 1: newState = PURPLE; break;
            case 2: newState = GREEN;  break;
            case 3: newState = EMPTY;  break;
            default: newState = UNKNOWN;
        }

        loadSlot = this.findCurrentSlotInPosition(LOAD);

        if(!loadSlot.contains(newState)) {
            loadSlot.setOccupied(newState);
            sortCooldown.reset();
        }
    }

    public boolean artifactSortCooldown() {
        return sortCooldown.seconds() > .5;
    }

    /**
     * Totals the number of Artifacts stored in the blender and updates the inventory class.
     */
    private void takeInventory() {
        int currentPurpleCount = 0;
        int currentGreenCount = 0;
        for (Slot currentSlot : allSlots) {
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
        inventory.updateInventory(currentPurpleCount, currentGreenCount);
    }

    /**
     * Searches the slots in order of ABC to find the first Slot containing the indicated contents.
     * @param slotType The target state of the Slot. Can be EMPTY, GREEN, PURPLE, or UNKNOWN
     *                 in the form of a SlotState enum.
     * @return The first found Slot filled with the indicated slotType.
     */
    public Slot findFirstType(SlotState slotType) {
        for (Slot currentSlot : allSlots) {
            if (currentSlot.contains(slotType)) {
                return currentSlot;
            }
        }
        return noSlot;
    }

    /**
     * searches for the nth ball of a certain type within the inventory.
     * Can be used to find a ball of type past the first of the type
     * @param slotType The target state of the Slot. Can be EMPTY, GREEN, PURPLE, or UNKNOWN
     *                 in the form of a SlotState enum.
     * @param nthBall The Artifact to search for
     * @return The found Slot
     */
    public Slot findXOfType(SlotState slotType, int nthBall)
    {
        int counter = 0;
        if((slotType == GREEN && inventory.greenCount >= nthBall) || (slotType == PURPLE && inventory.purpleCount >= nthBall)) {
            for (Slot currentSlot : allSlots) {
                if (currentSlot.contains(slotType)) {
                    counter++;
                    if (counter == nthBall) {
                        return currentSlot;
                    }
                }
            }
        }
        return noSlot;
    }


    /**
     * Searches the slots in order of ABC to find the first Slot containing an Artifact.
     * @return The first found Slot containing a ball
     */
    public Slot findFirstOccupied() {
        for (Slot currentSlot : allSlots) {
            if (currentSlot.contains(GREEN) || currentSlot.contains(PURPLE)) {
                return currentSlot;
            }
        }
        return noSlot;
    }


    /**
     * Searches the slots in order of ABC to find the first Slot that is known and does not contain
     * the specified contents. If the input is UNKNOWN, it will find the first known Slot.
     * @param slotType The not-target state of the Slot. Can be EMPTY, GREEN, PURPLE, or UNKNOWN in
     *                 the form of a SlotState enum.
     * @return The first found Slot.
     */
    public Slot findFirstNotType(SlotState slotType) {
        for (Slot currentSlot : allSlots) {
            if (currentSlot.doesNotContain(slotType, UNKNOWN)) {
                return currentSlot;
            }
        }
        return noSlot;
    }

    /**
     * Uses the current offset to find which Slot is in the specified Zone.
     * @param zone The target Zone.
     * @return The found Slot.
     */
    public Slot findSlotByZone(Zone zone) {
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

    /**
     * Finds a Slot in the FIRE or LOAD position, if there is one.
     * @param targetPosition FIRE or LOAD. FIRE is the position ready to launch, and LOAD is the
     *                       load position.
     * @return A Slot, if there's one in position. If there isn't, will return noSlot.
     */
    public Slot findCurrentSlotInPosition(BetaSorterHardware.positionState targetPosition) {
        Slot foundLoadSlot;
        Slot foundFireSlot;

        switch (getCurrentOffset()) {
            case 0:  foundLoadSlot = slotA; foundFireSlot = slotB; break;
            case 1:  foundLoadSlot = slotB; foundFireSlot = slotC; break;
            case 2:  foundLoadSlot = slotC; foundFireSlot = slotA; break;
            default: foundLoadSlot = noSlot; foundFireSlot = noSlot;
        }

        if (targetPosition == LOAD) {
            return foundLoadSlot;
        } else if (targetPosition == FIRE) {
            return foundFireSlot;
        } else return noSlot;
    }

    //TODO Move to LauncherHardware???
    /**
     * Uses the blender encoder and offsetPositions to calculate which offset position the blender
     * is currently in.
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
    private int findClosestOffset(int ticks) {
        ticks = equalizeMotorPositions(ticks);

        int lowestDistance = 1000000000;
        int offset = -2;
        int currentDistanceCheck;

        for(int i = 0; i < 6; i++) {
            currentDistanceCheck = Math.abs(offsetPositions.get(i) - ticks);
            if (currentDistanceCheck < lowestDistance) {
                lowestDistance = currentDistanceCheck;
                offset = i;
            }
        }
        // Check the high offset 0
        currentDistanceCheck = (int) Math.abs(BetaSorterHardware.ticksPerRotation - ticks);
        if (currentDistanceCheck < lowestDistance) {
            offset = 0;
        }

        return offset;
    }

    /**
     * Does some basic math to equalize a motor position between 0 and
     * SorterHardware.ticksPerRotation (8192).
     * @param ticks The number to be equalized.
     * @return The equalized position, between 0-8192.
     */
    public int equalizeMotorPositions(int ticks) {
        while (ticks > BetaSorterHardware.ticksPerRotation) {
            ticks -= BetaSorterHardware.ticksPerRotation;
        }
        while (ticks < 0) {
            ticks += BetaSorterHardware.ticksPerRotation;
        }
        return ticks;
    }

    public boolean isCurrentReferenceLogical(int reference) {
        return reference % ((double) BetaSorterHardware.ticksPerRotation / 6) == 0;
    }

    /**
     * A Slot is a representation of a physical Slot in the blender. The Slot class stores the
     * occupancy state (SlotState enum) and correct motor positions to load and fire an Artifact. A
     * constructor must be called to create an instance of the class, representing one slot in the
     * blender.
     */
    public class Slot {
        private SlotState occupied = UNKNOWN;
        private final int firePosition;
        private final int loadPosition;
        private final String name;

        /**
         * This constructor creates a Slot. A Slot is a representation of a physical Slot in the
         * blender.
         * @param motorLoadPosition The motor position to go to when firing, in ticks.
         * @param motorFirePosition The motor position to go to when loading, in ticks.
         */
        public Slot(int motorLoadPosition, int motorFirePosition, String name) {
            this.loadPosition = motorLoadPosition;
            this.firePosition = motorFirePosition;
            this.name = name;
        }

        /**
         * Returns the stored firing position for the Slot. If the Slot is noSlot,
         * will return the current blender reference, effectively avoiding any change.
         * @return The firing position, in ticks
         */
        public int getFirePosition() {
            return firePosition;
        }

        /**
         * Returns the stored loading position for the Slot. If the Slot is noSlot,
         * will return the current blender reference, effectively avoiding any change.
         * @return The loading position, in ticks
         */
        public int getLoadPosition() {
            return loadPosition;
        }

        /**
         * Returns the current occupation of the Slot. if the Slot is noSlot, will return UNKNOWN.
         * @return The current occupation of the Slot, in a SlotState enum.
         */
        public SlotState getOccupied() {
            return occupied;
        }

        /**
         * Stores the new contents in the Slot.
         * @param newOccupation The new SlotState
         */
        public void setOccupied(SlotState newOccupation) {
            occupied = newOccupation;
        }

        /**
         * Checks to see if the Slot contains the specified contents
         * @param checkState The contents to check for, in the form iof a SlotState enum.
         * @return Whether or not the Slot contains the contents checked for.
         */
        public boolean contains(SlotState checkState) {
            return occupied == checkState;
        }

        /**
         * Checks to see if the Slot doesn't contain any of the specified contents.
         * @param checkStates The state(s) to be checked against. Yes, state(s); it can handle
         *                    multiple.
         * @return Whether or not the Slot does not contain any of the contents checked against.
         */
        public boolean doesNotContain(SlotState... checkStates) {
            for (SlotState currentCheckState : checkStates) {
                if (currentCheckState == occupied) {
                    return false;
                }
            }
            return true;
        }

        /**
         * This function is an easy way to identify if a function returned a NoSlot.
         * @return A real Slot returns true, and a NoSlot returns false
         */
        public boolean exists() {
            return true;
        }

        /**
         * Gets a string for the name of the Slot: "A", "B", "C", or "No Slot Found".
         * @return The name of the Slot.
         */
        public String getName() {
            return name;
        }
    }

    /**
     * The NoSlot is a special case. It will create a "fake" Slot that always returns the
     * current blender reference for positions and UNKNOWN for occupancy. This is to ensure the
     * findFirstSlot(), findFirstNoSlot(), and findSlotByZone() functions can still return a "no
     * Slot found" option without returning null.
     */
    public class NoSlot extends Slot {
        /**
         * Creates a phantom Slot, a NoSlot, that does as little damage as possible when it's
         * returned. Positions will return the current blender reference and the occupancy is always
         * UNKNOWN.
         */
        NoSlot() { // Feed it junk data. If you ever see these numbers, we have a problem
            super(0, 0, "No Slot Found");
        }

        @Override
        public int getFirePosition() {
            return (int) robot.sorterHardware.reference;
        }

        @Override
        public int getLoadPosition() {
            return (int) robot.sorterHardware.reference;
        }

        @Override
        public void setOccupied(SlotState newOccupation) {}

        @Override
        public boolean contains(SlotState checkState) {
            return checkState == UNKNOWN;
        }

        @Override
        public boolean doesNotContain(SlotState... checkStates) {
            return true;
        }

        @Override
        public boolean exists() {
            return false;
        }
    }

    /**
     * This class is the inventory. It stores the current count of Artifacts,
     * for each color and in total, and whether or not a Pattern can be created.
     */
    public class SlotInventory {
        private int count;
        private int purpleCount;
        private int greenCount;
        public SlotInventory() {}

        public void updateInventory(int currentPurpleCount, int currentGreenCount) {
            setTotalCount(currentPurpleCount + currentGreenCount);
            setPurpleCount(currentPurpleCount);
            setGreenCount(currentGreenCount);
        }
        public int getTotalCount() {
            return count;
        }
        public void setTotalCount(int newCount) {
            count = newCount;
        }

        public int getPurpleCount() {
            return purpleCount;
        }
        public void setPurpleCount(int newPurpleCount) {
            purpleCount = newPurpleCount;
        }

        public int getGreenCount() {
            return greenCount;
        }
        public void setGreenCount(int newGreenCount) {
            greenCount = newGreenCount;
        }

        /**
         * Checks to see if two PURPLE Artifacts and one GREEN Artifact is in the inventory; in
         * other words, if we can fire one Pattern.
         * @return Whether or not we can make the Pattern
         */
        public boolean canMakePattern() {
            return greenCount == 1 && purpleCount == 2;
        }

    }

    /**
     * The Zone class is used to define and range of values for the camera to use. If the
     * center of a Blob is within the range (which is easy to check by calling the inRange() boolean
     * function), the Artifact represented by the Blob is within the Slot for which the range
     * represents.
     * <p>
     * This class also just represents the physical position of the blender if needed.
     */
    public class Zone {
        private final float xMin;
        private final float xMax;
        private final float yMin;
        private final float yMax;
        Zone(float xMin, float xMax, float yMin, float yMax) {
            this.xMin = xMin;
            this.xMax = xMax;
            this.yMin = yMin;
            this.yMax = yMax;
        }

        /**
         * The inRange() function checks to see if a given coordinate is within the Zone. Its
         * intended implementation is to check whether or not an Artifact, represented by a Blob, is
         * within a Slot, represented by a Zone. Input the center coordinates of a Blob to test
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