package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class SorterHardware {

    public int currentTickCount;
    public boolean currentlyMoving = false;
    public boolean readyForNextTick = false;
    public boolean inMagPosition;
    public boolean inPosition;
    public int targetClicks;
    public int tickTollerance = 5;

    public int[] positions;

    public int ticksPerRotation = 8192;

    public Basic_Strafer_Bot disRobot;
    public DcMotorEx motor;

    public void initDaSorter(DcMotorEx motor, Basic_Strafer_Bot robot)
    {
        disRobot = robot;
        //motor = robot.sorterMotor;

        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        motor.setPositionPIDFCoefficients(1.0); // 1.0 default

        positions[0] = 0; //Slot one load
        positions[1] = (ticksPerRotation / 2);//Slot one launch
        positions[2] = (ticksPerRotation/3); //Slot two load
        positions[3] = (ticksPerRotation/3 + (ticksPerRotation/2)); // slot two launch
        positions[4] = 2*(ticksPerRotation/3);//Slot three load
        positions[5] = (2*(ticksPerRotation/3)) + (ticksPerRotation/2);//Slot three launch
    }

    public int findMagsToTarget(int currentSlot, int targetSlot)
    {
        return Math.abs(targetSlot - currentSlot);
    }

    public boolean countToTarget(boolean startingOnPosition)// We can clean this up once the robot files are merged
    {//returns true when at target until other vars are reset

            if(startingOnPosition)
            {
                targetClicks++;
            }

            if (disRobot.magsense.getValue() == 0) {
                readyForNextTick = true;
            } else if (disRobot.magsense.getValue() == 1 && readyForNextTick) {
                currentTickCount++;
                readyForNextTick = false;
            }
        inMagPosition = currentTickCount == targetClicks;
        return inMagPosition;

    }




    public int findFastestRotationInTicks(int currentPosition, int targetPosition)
    {
        //Finds the shortest route to the slot position reguardless of how high/low we go


        int howManyCycles = (currentPosition / ticksPerRotation);

        int slotSpaces[] = new int[0];



        slotSpaces[0] = targetPosition * (howManyCycles-1);
        slotSpaces[1] = targetPosition * (howManyCycles+1);
        slotSpaces[3] = targetPosition * howManyCycles;
        slotSpaces[4] = targetPosition;

        int currentlowest = slotSpaces[0];

        for(int i = 0; i<4; i++)
        {
           if (Math.abs((slotSpaces[i])-currentPosition) < currentlowest)
           {
               currentlowest = slotSpaces[i];
           }
        }

        return currentlowest;
    }

    public boolean inPropperTickPositon()
    {
        if(motor.getCurrentPosition() > motor.getTargetPosition() - tickTollerance  &&  motor.getCurrentPosition() < motor.getTargetPosition() + tickTollerance)
        {
            return true;
        }
        else return false;
    }

    public boolean safeCheck()
    {
        if(inMagPosition && inPropperTickPositon())
        {
            currentlyMoving = false;
            return true;
        }
        else return  false;
    }

    public void prepareNewMovement(int currentTickPose, int targetTickPose, int currentSlot, int targetSlot)
    {
        currentTickCount = 0;
        targetClicks = findMagsToTarget(currentSlot, targetSlot);
        motor.setTargetPosition(findFastestRotationInTicks(currentTickPose, targetTickPose));
    }

    public void spin()
    {
        motor.setPower(0.5);
        motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        currentlyMoving = true;
    }















}