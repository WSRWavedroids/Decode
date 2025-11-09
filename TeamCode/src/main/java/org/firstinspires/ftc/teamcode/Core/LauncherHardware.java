package org.firstinspires.ftc.teamcode.Core;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
public class LauncherHardware {

    public boolean inSpeedRange;

    public Basic_Strafer_Bot disBot;
    public DcMotorEx motor;

    public boolean on = false;
    public boolean waitingForServo = false;
    public boolean waitingToFire;

    private Robot robot;

    public double toleranceRange = 5;

    public double speedTarget;

    public double distanceMultiplier;

    public Servo hammerServo;

    private ElapsedTime cooldownTimer = new ElapsedTime();
    private double timeForMove = 0.3;
    public boolean onCooldown = false;

    private boolean hammerForward;
    private boolean hammerBack;
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
        motor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
        hammerServo = robot.hammerServo;
        hammerServo.setPosition(hammerBackPosition);
    }

    public boolean motorSpeedCheck(double speedTarget) {
        //robot.launcher.rampSpeed(2);
        return (motor.getVelocity() > speedTarget - toleranceRange) && (motor.getVelocity() < speedTarget + toleranceRange);
    }

    public double findSpeed(double distance) {
        return distance * distanceMultiplier;
    }

    public void readyFire() {
        waitingToFire = true;
    }


    public void fire() {
        waitingToFire = false;
        runHammer();
        onCooldown = true;
        cooldownTimer.reset();

    }

    public void runHammer() {

        if(onCooldown && hammerBack)
        {
            hammerServo.setPosition(hammerForwardPosition);
        }

        else if(onCooldown && hammerForward)
        {
            hammerServo.setPosition(hammerBackPosition);
        }
    }

    public void timerCheck()
    {
        if(cooldownTimer.seconds() >= timeForMove)
        {
            hammerBack = false;
            hammerForward = true;
        }
        else if(cooldownTimer.seconds() >= timeForMove *2)
        {
            hammerBack = true;
            hammerForward = false;
            onCooldown = false;

            if(firingInSequence)
            {
                //robot.sorterHardware.nextPhaseStep();
            }
        }
    }


    public void setLauncherSpeed(double targetspeed) {
        motor.setVelocity(ticksPerRevolution * (revolutionsPerSecond * targetspeed));
    }

    public void updateLauncherHardware() {
        inSpeedRange = motorSpeedCheck(speedTarget);
        timerCheck();

        if(inSpeedRange)
        {
            spikeable = true;
            spikeableValue = motor.getVelocity();
        }

        if (inSpeedRange && robot.sorterHardware.fireSafeCheck() && waitingToFire) {
            fire();
        }

    }

    public boolean spikeable = false;
    public double spikeableValue;

    public void deacelerationDetection()
    {

    }

}