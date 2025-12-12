package org.firstinspires.ftc.teamcode.Core;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.*;
import static org.firstinspires.ftc.teamcode.Core.ArtifactLocator.SlotState.*;
import static org.firstinspires.ftc.teamcode.Core.SorterHardware.positionState.*;
import static org.firstinspires.ftc.teamcode.Core.fireQueue.firingQueue.SMART;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
public class LauncherHardware {

    public boolean inSpeedRange;

    public Basic_Strafer_Bot disBot;
    public DcMotorEx motor;

    public boolean on = false;
    public boolean waitingForServo = false;
    public boolean waitingToFire = false;

    public boolean wantToOpenDoor;

    protected Robot robot;

    public double toleranceRange = 350;

    public double velocityTarget;

    public double distanceMultiplier;

    public Servo hammerServo;

    public ElapsedTime cooldownTimer = new ElapsedTime();
    public static double launcherCooldownDuration = 0.25/2;
    public boolean onCooldown = false;

    private boolean hammerForward;
    private boolean hammerBack = true;
    public double hammerForwardPosition = 0;
    public double hammerBackPosition = .25;

    public boolean firingInSequence;

    public int ticksPerRevolution = 28;
    public int revolutionsPerSecond = 100;

    public double P;
    public double I;
    public double D;
    public double F;

    public LauncherHardware(Robot robotFile) {
        robot = robotFile;
        motor = robot.launcherMotor;
        // motor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
        motor.setDirection(REVERSE);
        hammerServo = robot.hammerServo;
        waitingToFire = false;
    }

    public boolean motorSpeedCheck(double speedTarget) {
        //robot.launcher.rampSpeed(2);
        return (motor.getVelocity() > (-speedTarget - toleranceRange)) && (motor.getVelocity() < (-speedTarget + toleranceRange));
    }

    public double findSpeed(double distance) {
        return distance * distanceMultiplier;
    }

    public void readyFire() {
        readyFire(0, false);
    }

    public void readyFire(double speedTarget, boolean useSpeedTarget) {
        waitingToFire = true;
        if(useSpeedTarget)
        {
            setLauncherSpeed(speedTarget);
        }
    }

    public void fire() {
        waitingToFire = false;
        onCooldown = true;
        cooldownTimer.reset();
        wantToOpenDoor = true;
    }

    public void runHammer() {

        if(onCooldown && hammerBack)
        {
            hammerServo.setPosition(hammerBackPosition);
            //robot.sorterHardware.doorServo.setPosition(robot.sorterHardware.doorClosedPosition);
        }

        else if(onCooldown && hammerForward)
        {
            hammerServo.setPosition(hammerBackPosition);
        }
    }

    public void timerCheck()
    {
        robot.telemetry.addLine("Checking the timer");

        if (cooldownTimer.seconds() > (launcherCooldownDuration * 2)) //if hammer timer is over we know we done firing
        {
            hammerBack = true;
            hammerForward = false;
            onCooldown = false;
            wantToOpenDoor = false;
        }
        else if(cooldownTimer.seconds() > launcherCooldownDuration)
        {
            hammerBack = false;
            hammerForward = true;
        }

    }


    public void setLauncherSpeed(double targetspeed) {
        velocityTarget = ticksPerRevolution * (revolutionsPerSecond * targetspeed);
        motor.setVelocity(velocityTarget);
    }

    public void updateLauncherHardware() {
        inSpeedRange = motorSpeedCheck(velocityTarget);
        timerCheck();
        runHammer();

        if (robot.sorterHardware.fireSafeCheck() && waitingToFire) {
            fire();
        }

        if (hammerForward) {
            robot.sorterLogic.findCurrentSlotInPosition(FIRE).setOccupied(EMPTY);
        }

        if (robot.queue.wantToFireQueue == SMART) {
            robot.queue.fireAllSmart(1, true);
        }
    }

    public boolean spikeable = false;
    public double spikeableValue;

    public void decelerationDetection()
    {

    }

}