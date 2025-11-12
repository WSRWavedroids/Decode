package org.firstinspires.ftc.teamcode.Core;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.Core.Robot.openClosed.CLOSED;
import static org.firstinspires.ftc.teamcode.Core.Robot.openClosed.OPEN;

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
    public boolean waitingToFire;

    private Robot robot;

    public double toleranceRange = 350;

    public double velocityTarget;

    public double distanceMultiplier;

    public Servo hammerServo;

    private ElapsedTime cooldownTimer = new ElapsedTime();
    private double cooldownDuration = 0.3;
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
    }

    public boolean motorSpeedCheck(double speedTarget) {
        //robot.launcher.rampSpeed(2);
        return (motor.getVelocity() > (-speedTarget - toleranceRange)) && (motor.getVelocity() < (-speedTarget + toleranceRange));
    }

    public double findSpeed(double distance) {
        return distance * distanceMultiplier;
    }

    public void readyFire(double speedTarget) {
        waitingToFire = true;
        setLauncherSpeed(speedTarget);
    }

    public void fire() {
        waitingToFire = false;
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
        robot.telemetry.addLine("Checking the timer");

        if (cooldownTimer.seconds() >= (cooldownDuration * 2))
        {
            hammerBack = true;
            hammerForward = false;
            onCooldown = false;
        }
        else if(cooldownTimer.seconds() >= cooldownDuration)
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


        if(inSpeedRange)
        {
            spikeable = true;
            spikeableValue = motor.getVelocity();
        }
        if (robot.sorterHardware.fireSafeCheck() && waitingToFire && inSpeedRange) {
            robot.sorterHardware.triggerServo(OPEN);
        }
        else if (!robot.launcher.onCooldown) {
            robot.sorterHardware.triggerServo(CLOSED);
        }

        if (robot.sorterHardware.fireSafeCheck() && waitingToFire && robot.sorterHardware.openCheck()) {
            fire();
        }
    }

    public boolean spikeable = false;
    public double spikeableValue;

    public void decelerationDetection()
    {

    }

}