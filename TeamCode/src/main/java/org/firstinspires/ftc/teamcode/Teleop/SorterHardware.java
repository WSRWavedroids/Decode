package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.hardware.TouchSensor;

public class SorterHardware {

    public static int currentTickCount;
    public boolean currentlyCounting = false;
    public static boolean readyForNextTick = false;
    public boolean inPosition;
    public static int targetClicks;

    //TODO fill in with correct positions
    public static double slotAIntake;
    public static double slotALaunch;

    public static double slotBIntake;
    public static double slotBLaunch;

    public static double slotCIntake;
    public static double slotCLaunch;

    public int findMagsToTarget(int currentOffset, int targetOffset)
    {
        return Math.abs(targetOffset - currentOffset);
    }

    public static boolean countToTarget(TouchSensor sensor, boolean startingOnPosition)
    {

            if(startingOnPosition)
            {
                targetClicks++;
            }

            if (sensor.getValue() == 0) {
                readyForNextTick = true;
            } else if (sensor.getValue() == 1 && readyForNextTick) {
                currentTickCount++;
                readyForNextTick = false;
            }

        return currentTickCount == targetClicks;
    }

    public void resetCount()
    {
        currentTickCount = 0;
    }

    public void prepareNewMovement(int currentSlot, int targetSlot)
    {
        resetCount();
        targetClicks = findMagsToTarget(currentSlot, targetSlot);
        currentlyCounting = true;
    }







}