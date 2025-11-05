package org.firstinspires.ftc.teamcode.Core;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
public class SorterHardware {
    public boolean currentlyMoving = false;

    public boolean inMagPosition;

    public int currentTickCount;
    public static   int tickTolerance = 5;
    public int[] positions;
    public int ticksPerRotation = 8192;
    public static int offset = 0;
    public boolean legalToSpin = false;


    public Servo doorServo;

    private CRServo feedServoL;
    private CRServo feedServoR;
    public double feederSpeed = 0.5;
    public boolean tryingToFeed = false;

    public boolean open = false;
    public boolean wantToMoveDoor = false;
    public double doorClosedPosition = 1;
    public double doorOpenPosition = 0.75;

    public Robot disRobot;
    public DcMotorEx motor;
    public LauncherHardware launcher;

    String doorTarget;

    private ElapsedTime cooldownTimer = new ElapsedTime();
    private boolean onCooldown = false;
    private double cooldownDuration = 0.5;

    public static double kp = 0.00029;

    public static double ki = 0.000006;//maybe try 275 where 275 is

    public static double kd = 0.000004;
    public static double kf = 0.0;

    public ElapsedTime pidfTime() {
        return cooldownTimer;
    }


    public double reference;

    double integralSum = 0;

    double lastError = 0;

    public SorterHardware(Robot robot)
    {
        disRobot = robot;
        motor = robot.sorterMotor;
        doorServo = robot.doorServo;
        launcher = robot.launcher;
        feedServoL = robot.feedServoL;
        feedServoR = robot.feedServoR;


        positions = new int[6];
        positions[0] = 0; //Slot one load
        positions[1] = (ticksPerRotation / 2);//Slot one launch
        positions[2] = (ticksPerRotation/3); //Slot two load
        positions[3] = (ticksPerRotation/3 + (ticksPerRotation/2)); // slot two launch
        positions[4] = 2*(ticksPerRotation/3);//Slot three load
        positions[5] = (2*(ticksPerRotation/3)) + (ticksPerRotation/2); //Slot three launch

        doorServo.setPosition(doorClosedPosition);

        reference = findFastestRotationInTicks(motor.getCurrentPosition(), positions[0]);

        //inMagPosition = true;
    }



    public boolean getLimitSwitch()
    {
        return !disRobot.magsense.getState(); //defaults to true if empty so flip
    }

    public void runFeeders(double speed)
    {
        feedServoR.setPower(speed);
        feedServoL.setPower(-speed);
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

    public boolean inProperTickPosition()
    {
        if(motor.getCurrentPosition() > motor.getTargetPosition() - tickTolerance &&  motor.getCurrentPosition() < motor.getTargetPosition() + tickTolerance)
        {
            return true;
        }
        else return false;
    }

    public boolean positionedCheck()
    {
        if(/*getLimitSwitch() && */ inProperTickPosition())
        {
            currentlyMoving = false;
            return true;
        }
        else return false;
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

        if(!disRobot.launcher.onCooldown && !positionedCheck() && closedCheck() && legalToSpin) //if not on servo timeout and not already there, rotate
        {
            return true;
        }else
        {
            return false;
        }
    }

    public boolean fireSafeCheck()
    {

        //if not on servo timeout and there and open, fire
        return !disRobot.launcher.onCooldown && positionedCheck() && openCheck();
    }

    public void prepareNewMovement(int currentTickPose, int targetTickPose/*, int currentSlot, int targetSlot*/)
    {
        reference = (findFastestRotationInTicks(currentTickPose, targetTickPose)) + offset;
    }

    public void spin()
    {
            runPIDMotorStuffLol();
            currentlyMoving = true;
    }
    
    public void updateSorterHardware()
    {
        if(moveSafeCheck())
        {
            spin();
        }
        else
        {
            Estop();
        }


        if(positionedCheck() && wantToMoveDoor)
        {
            moveDoor();
        }

        if(positionedCheck() && tryingToFeed)
        {
            runFeeders(feederSpeed);
        }
        else {
            runFeeders(0);
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

        double feedforward = kf * reference;

        // sum of all error over time
        integralSum = integralSum + (error * pidfTime().seconds());

        double out = (kp * error) + (ki * integralSum) + (kd * derivative) + feedforward;

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(out);

        lastError = error;

        // reset the timer for next time
        pidfTime().reset();

    }



}