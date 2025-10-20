package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.opencv.core.Mat;

public class SorterHardware {

    public int currentTickCount;
    public boolean currentlyCounting = false;
    public boolean readyForNextTick = false;
    public boolean inPosition;
    public int targetClicks;

    public double slotOneIntake;
    public double slotOneLaunch;

    public double slotTwoIntake;
    public double slotTwoLaunch;

    public double slotThreeIntake;
    public double slotThreeLaunch;

    public double motorTargetSpeed;
    public boolean motorSpeedOK;


    public int findMagsToTarget(int currentSlot, int targetSlot)
    {
        return Math.abs(targetSlot - currentSlot);
    }

    public boolean countToTarget(TouchSensor sensor, boolean startingOnPosition)
    {//returns true when at target until other vars are reset

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
    }

    public boolean motorSpeedCheck(DcMotorEx motor, double speedTarget, double tolleranceRange)
    {

        if((motor.getVelocity() > motor.getVelocity()-tolleranceRange) && (motor.getVelocity() < motor.getVelocity()+tolleranceRange))
        {
            return true;
        }
        else
        {
            return false;
        }
    }











}