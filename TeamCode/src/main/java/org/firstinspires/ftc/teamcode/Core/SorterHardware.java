package org.firstinspires.ftc.teamcode.Core;

import static org.firstinspires.ftc.teamcode.Core.Robot.openClosed.CLOSED;
import static org.firstinspires.ftc.teamcode.Core.Robot.openClosed.OPEN;
import static org.firstinspires.ftc.teamcode.Core.SorterHardware.positionState.FIRE;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
public class SorterHardware {
    public boolean currentlyMoving = false;
    public enum positionState {FIRE, LOAD, SWITCH}
    public positionState currentPositionState;
    public boolean inMagPosition;

    public int currentTickCount;
    public static int tickTolerance = 100;
    public int[] positions;
    public int lastSafePosition;
    public int ticksPerRotation = 8192;
    public static int offset = 0;
    public boolean legalToSpin = false;


    public Servo doorServo;

    private CRServo feedServoL;
    private CRServo feedServoR;
    public double feederIntakeSpeed = 1;
    public double feederRotateSpeed = 1;
    public double passiveFeederSpeed = 1;
    public boolean tryingToFeed = false;

    public boolean open = false;
    public boolean wantToMoveDoor = false;
    public double doorClosedPosition = 1;
    public double doorOpenPosition = 0.75;

    public boolean alreadyClosed;

    public Robot disRobot;
    public DcMotorEx motor;
    public LauncherHardware launcher;

    public Robot.openClosed doorTarget = CLOSED;

    public ElapsedTime cooldownTimer = new ElapsedTime();
    public boolean onCooldown = false;
    private double cooldownDuration = 0.25;

    private ElapsedTime pidfTimer = new ElapsedTime();

    public static double kneecap = .4;
    public static double kp = 0.001;
    public static double ki = 0.0000002;//maybe try 275 where 275 is
    public static double kd = 0.0;
    public static double kf = 0.0;

    public ElapsedTime pidfTime() {
        return pidfTimer;
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
        positions[0] = 0; //Slot A load
        positions[1] = ticksPerRotation / 2; //Slot A launch
        positions[2] = (2 * ticksPerRotation/3); //Slot B load
        positions[3] = ticksPerRotation / 6; // slot B launch
        positions[4] = ticksPerRotation / 3; //Slot C load
        positions[5] = 5 * ticksPerRotation / 6; //Slot C launch

        reference = 0;
    }



    public boolean getLimitSwitch()
    {
        return !disRobot.magsense.getState(); //defaults to true if empty so flip
    }

    public void runFeeders(double speed)
    {
        feedServoR.setPower(-speed);
        feedServoL.setPower( speed);
    }

    public int findFastestRotationInTicks(int currentPosition, int targetPosition) {
        int howManyCycles = currentPosition / ticksPerRotation;

        int[] slotSpaces = new int[3];
        slotSpaces[0] = targetPosition + (howManyCycles - 1) * ticksPerRotation;
        slotSpaces[1] = targetPosition + howManyCycles * ticksPerRotation;
        slotSpaces[2] = targetPosition + (howManyCycles + 1) * ticksPerRotation;

        int bestPosition = slotSpaces[0];
        int smallestDistance = Math.abs(slotSpaces[0] - currentPosition);

        for (int i = 1; i < 3; i++) {
            int distance = Math.abs(slotSpaces[i] - currentPosition);
            if (distance < smallestDistance) {
                smallestDistance = distance;
                bestPosition = slotSpaces[i];
            }
        }

        return bestPosition;
    }

    public boolean inProperTickPosition()
    {
        if(motor.getCurrentPosition() > reference - tickTolerance &&  motor.getCurrentPosition() < reference + tickTolerance)
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

    public void triggerServo(Robot.openClosed goTo)
    {
        doorTarget = goTo;
        wantToMoveDoor = true;
    }

    public void moveDoor()//Needs to become a state in update
    {
            wantToMoveDoor = false;


            disRobot.telemetry.addData("Setting door to", doorTarget);

            if(doorTarget == CLOSED)
            {
                doorServo.setPosition(doorClosedPosition);
                open = false;
                alreadyClosed = true;

                //door
            }
            if(doorTarget == OPEN)
            {
                doorServo.setPosition(doorOpenPosition);
                open = true;
                alreadyClosed = false;
                //cooldownTimer.reset();

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

    public boolean inStateCheck(positionState targetState){
        return currentPositionState == targetState;
    }

    public boolean moveSafeCheck()
    {
        //if not on servo timeout and not already there, rotate
        return !disRobot.launcher.onCooldown && !positionedCheck() && closedCheck() && legalToSpin;
    }

    public boolean fireSafeCheck()
    {
        //if not on servo timeout and there and open, fire
        return positionedCheck()  && inStateCheck(FIRE);
    }

    public void prepareNewMovement(int currentTickPose, int targetTickPose/*, int currentSlot, int targetSlot*/)
    {
        lastSafePosition = currentTickPose;
        reference = (findFastestRotationInTicks(currentTickPose, targetTickPose));
        disRobot.telemetry.addLine("New position");

    }

    public void spin()
    {
            runPIDMotorStuffLol();
            currentlyMoving = true;
            disRobot.telemetry.addLine("Moving");
    }
    
    public void updateSorterHardware()
    {

        switch (disRobot.sorterLogic.getCurrentOffset()) {
            // Firing positions
            case 1: currentPositionState = FIRE; break;
            case 3: currentPositionState = FIRE; break;
            case 5: currentPositionState = FIRE; break;

            // Loading positions
            case 0: currentPositionState = positionState.LOAD; break;
            case 2: currentPositionState = positionState.LOAD; break;
            case 4: currentPositionState = positionState.LOAD; break;

            // Not in a position (-1)
            default: currentPositionState = positionState.SWITCH;
        }

        if(moveSafeCheck())
        {
            spin();
        }
        else
        {
            Estop();
        }



        if(cooldownTimer.seconds() > cooldownDuration)
        {
            onCooldown = false;

        }
        //Move the door automatically when requested and safe
        if (positionedCheck())
        {
            if(!onCooldown && !alreadyClosed)
            {
                triggerServo(CLOSED);
            }
            if (fireSafeCheck() && disRobot.launcher.wantToOpenDoor) {
                triggerServo(OPEN);
            }
            if(wantToMoveDoor && fireSafeCheck())
            {
                if(doorTarget == OPEN)
                {
                    triggerDoorCooldown();
                }
                moveDoor();
            }
        }





        if(tryingToFeed)
        {
            runFeeders(feederIntakeSpeed);
        } else if (moveSafeCheck()) {
            runFeeders(feederRotateSpeed);
        } else {
            runFeeders(passiveFeederSpeed);
        }



        //jam detection

        if(jamDetection())
        {
            disRobot.telemetry.addData("Jammed ", ":(");
            //prepareNewMovement(motor.getCurrentPosition(), lastSafePosition);
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

        double out = kneecap * ((kp * error) + (ki * integralSum) + (kd * derivative) + feedforward);

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(out);

        lastError = error;

        // reset the timer for next time
        pidfTime().reset();

    }

    public void resetSorterEncoder()
    {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public boolean jamDetection()
    {
        return motor.getPower() > (kneecap * 0.35) && motor.getVelocity() < 300 && !positionedCheck();
    }

    public void triggerDoorCooldown()
    {
        onCooldown = true;
        cooldownTimer.reset();
    }



}