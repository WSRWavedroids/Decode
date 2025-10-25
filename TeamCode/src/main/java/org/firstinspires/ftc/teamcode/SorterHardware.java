package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;

@Configurable
public class SorterHardware {
    public boolean currentlyMoving = false;

    public boolean readyForNextTick = false;
    public boolean inMagPosition;
    public int targetClicks;

    public int currentTickCount;
    public int tickTolerance = 5;
    public int[] positions;
    public int ticksPerRotation = 8192;


    public Servo doorServo;
    public boolean open;
    public boolean wantToMoveDoor;
    public double doorClosedPosition = 0;
    public double doorOpenPosition = 1;

    public Robot disRobot;
    public DcMotorEx motor;
    public LauncherHardware launcher;

    String doorTarget;

    private ElapsedTime cooldownTimer = new ElapsedTime();
    private boolean onCooldown = false;
    private double cooldownDuration = 0.5;

    public static double kp = 0.00012;
    public static double ki = 0.0;
    public static double kd = 0.0;

    public ElapsedTime pidfTime() {
        return cooldownTimer;
    }


    double reference;

    double integralSum = 0;

    double lastError = 0;

    public SorterHardware(Robot robot)
    {
        disRobot = robot;
        motor = robot.sorterMotor;
        doorServo = robot.doorServo;
        launcher = robot.launcher;


        //motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        motor.setTargetPositionTolerance(10);

        positions = new int[6];
        positions[0] = 0; //Slot one load
        positions[1] = (ticksPerRotation / 2);//Slot one launch
        positions[2] = (ticksPerRotation/3); //Slot two load
        positions[3] = (ticksPerRotation/3 + (ticksPerRotation/2)); // slot two launch
        positions[4] = 2*(ticksPerRotation/3);//Slot three load
        positions[5] = (2*(ticksPerRotation/3)) + (ticksPerRotation/2); //Slot three launch

        doorServo.setPosition(doorClosedPosition);
        reference = findFastestRotationInTicks(motor.getCurrentPosition(), 0);
        //motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

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

    public void triggerServo(String goTo)
    {
        doorTarget = goTo;
        wantToMoveDoor = true;
    }

    public void moveDoor()//Needs to become a state in update
    {
            wantToMoveDoor = false;
            cooldownTimer.reset();
            onCooldown = true;

            if(doorTarget.equals("CLOSED"))
            {
                doorServo.setPosition(doorClosedPosition);
                open = false;
                //door
            }
            if(doorTarget.equals("OPEN"))
            {
                doorServo.setPosition(doorOpenPosition);
                open = true;
            }
    }

    public boolean closedCheck()
    {
        return !onCooldown && !open;
    }

    public boolean openCheck()
    {
        return !onCooldown && open;
    }

    public boolean moveSafeCheck()
    {

        if(!disRobot.launcher.onCooldown && !positionedCheck() && closedCheck()) //if not on servo timeout and not already there, rotate
        {
            return true;
        }else
        {
            return false;
        }
    }

    public boolean fireSafeCheck()
    {

        if(!disRobot.launcher.onCooldown && !positionedCheck() && openCheck()) //if not on servo timeout and not already there, rotate
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


        reference = (findFastestRotationInTicks(currentTickPose, targetTickPose));

    }

    public void spin()
    {
            //motor.setPower(0.3);
            //motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            runPIDMotorStuffLol();
            currentlyMoving = true;
    }
    
    public void updateSorterHardware()
    {

        countMagsToTarget();

        if(moveSafeCheck())
        {
            spin();
        }
        else
        {
            Estop();
        }

        if(wantToMoveDoor && positionedCheck())
        {
            moveDoor();
        }
    }
    
    public void Estop()
    {
        motor.setPower(0);
    }

    public void runPIDMotorStuffLol()
    {
        // obtain the encoder position
        double encoderPosition = motor.getCurrentPosition();
        // calculate the error
        double error = reference - encoderPosition;

        // rate of change of the error
        double derivative = (error - lastError) / pidfTime().seconds();

        // sum of all error over time
        integralSum = integralSum + (error * pidfTime().seconds());

        double out = (kp * error) + (ki * integralSum) + (kd * derivative);

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(out);

        lastError = error;

        // reset the timer for next time
        pidfTime().reset();

    }

}