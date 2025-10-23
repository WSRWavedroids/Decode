package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
public class SorterHardware {

    public int currentTickCount;
    public boolean currentlyMoving = false;
    public boolean safe;
    public boolean readyForNextTick = false;
    public boolean inMagPosition;
    public boolean inPosition;
    public int targetClicks;
    public int tickTolerance = 5;

    public int[] positions;

    public int ticksPerRotation = 8192;

    public Basic_Strafer_Bot disRobot;
    public DcMotorEx motor;
    public LauncherHardware launcher;

    private ElapsedTime cooldownTimer = new ElapsedTime();
    private boolean onCooldown = false;
    private double cooldownDuration = 0.5;

    public void initDaSorter(Basic_Strafer_Bot robot, LauncherHardware launcherh)
    {
        disRobot = robot;
        motor = robot.sorterMotor;
        launcher = launcherh;
        //motor = robot.sorterMotor;

        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        motor.setPositionPIDFCoefficients(1.0); // 1.0 default

        positions = new int[6];
        positions[0] = 0; //Slot one load
        positions[1] = (ticksPerRotation / 2);//Slot one launch
        positions[2] = (ticksPerRotation/3); //Slot two load
        positions[3] = (ticksPerRotation/3 + (ticksPerRotation/2)); // slot two launch
        positions[4] = 2*(ticksPerRotation/3);//Slot three load
        positions[5] = (2*(ticksPerRotation/3)) + (ticksPerRotation/2); //Slot three launch
        
        motor.setTargetPosition(0);
        //motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        inMagPosition = true;
        
    }

    public int findMagsToTarget(int currentSlot, int targetSlot)
    {
        return Math.abs(targetSlot - currentSlot);
    }

    public boolean countMagsToTarget()
    {//returns true when at target until other vars are reset
        
            if (disRobot.magsense.getValue() == 0) {
                readyForNextTick = true;
            } else if (disRobot.magsense.getValue() == 1 && readyForNextTick) {
                currentTickCount++;
                readyForNextTick = false;
            }
        inMagPosition = currentTickCount == targetClicks;
        return inMagPosition;

    }
    
    public void resetMagCountAndTarget(boolean startOnPosition)
    {
        currentTickCount = 0;
        
        if(startOnPosition) 
        {
            targetClicks++;
        }
    }

    public int findFastestRotationInTicks(int currentPosition, int targetPosition)
    {
        //Finds the shortest route to the slot position reguardless of how high/low we go

        int howManyCycles = (currentPosition / ticksPerRotation);

        int[] slotSpaces = new int[3];
        slotSpaces[0] = targetPosition + (howManyCycles - 1) * ticksPerRotation;
        slotSpaces[1] = targetPosition + howManyCycles * ticksPerRotation;
        slotSpaces[2] = targetPosition + (howManyCycles + 1) * ticksPerRotation;

        int currentLowest = slotSpaces[0];

        for(int i = 0; i<3; i++)
        {
           if (Math.abs(slotSpaces[i]-currentPosition) < currentLowest)
           {
               currentLowest = slotSpaces[i];
           }
        }

        return currentLowest;
    }

    public boolean inPropperTickPositon()
    {
        if(motor.getCurrentPosition() > motor.getTargetPosition() - tickTolerance &&  motor.getCurrentPosition() < motor.getTargetPosition() + tickTolerance)
        {
            return true;
        }
        else return false;
    }

    public boolean positionedCheck()
    {
        if(/*inMagPosition &&*/ inPropperTickPositon())
        {
            currentlyMoving = false;
            return true;
        }
        else return  false;
    }

    /*public void triggerServoCooldown()
    {
        cooldownTimer.reset();
        onCooldown = true;
    }*/

    public boolean finalSafeCheck()
    {
        /*if(onCooldown)
        {
            if(cooldownTimer.seconds() >= cooldownDuration)
            {
                onCooldown = false;
            }
        }*/

        if(!launcher.waitingForServo && !positionedCheck()) //if not on servo timeout and not already there, rotate
        {
            return true;
        }else
        {
            return false;
        }
    }

    public void prepareNewMovement(int currentTickPose, int targetTickPose/*, int currentSlot, int targetSlot*/)
    {
        //resetMagCountAndTarget(stoppedCheck());
        //targetClicks = findMagsToTarget(currentSlot, targetSlot);


        motor.setTargetPosition(findFastestRotationInTicks(currentTickPose, targetTickPose));

    }

    public void spin()
    {
            motor.setPower(0.25);
            motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            currentlyMoving = true;
    }
    
    public void updateSorterHardware()
    {

        countMagsToTarget();

        if(finalSafeCheck())
        {
            spin();
        }
        else
        {
            Estop();
        }
    }
    
    public void Estop()
    {
        motor.setPower(0);
    }

}