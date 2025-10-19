package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.opencv.core.Mat;

public class SorterHardware {

    public int currentTickCount;
    public boolean currentlyCounting = false;
    public boolean readyForNextTick = false;

    public double slotOneIntake;
    public double slotOneLaunch;

    public double slotTwoIntake;
    public double slotTwoLaunch;

    public double slotThreeIntake;
    public double slotThreeLaunch;


    public int findMagsToTarget(int currentSlot, int targetSlot)
    {
        return Math.abs(targetSlot - currentSlot);
    }

    public boolean countToTarget(int clicks, TouchSensor sensor, boolean startingOnPosition)
    {

            if(startingOnPosition)
            {
                clicks++;
            }

            if (sensor.getValue() == 0) {
                readyForNextTick = true;
            } else if (sensor.getValue() == 1 && readyForNextTick) {
                currentTickCount++;
                readyForNextTick = false;
            }

        return currentTickCount == clicks;

    }

    public void resetCount()
    {
        currentTickCount = 0;
    }









}