package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
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

    private ElapsedTime cooldownTimer;
    private double timeForMove;
    private boolean onCooldown;

    private boolean hammerForward;
    private boolean hammerBack;


    public double P;
    public double I;
    public double D;
    public double F;

    public LauncherHardware(Robot robotFile) {
        robot = robotFile;
        motor = disBot.launcherMotor;
        motor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
        hammerServo = robot.hammerServo;
    }


    public boolean motorSpeedCheck(double speedTarget) {
        if ((motor.getVelocity() > speedTarget - toleranceRange) && (motor.getVelocity() < speedTarget + toleranceRange)) {
            robot.launcher.rampSpeed(2);
            return true;
        } else {
            return false;
        }
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
        cooldownTimer.reset();
    }

    public void runHammer() {

        if(onCooldown && hammerBack)
        {
            hammerServo.setPosition(1);
        }

        else if(onCooldown && hammerForward)
        {
            hammerServo.setPosition(0);
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
        }

    }






    public void rampSpeed(double targetspeed) {
        motor.setVelocity(targetspeed);
    }

    public void cutSpeed() {
        motor.setVelocity(0);
    }

    public void updateLauncherHardware() {
        inSpeedRange = motorSpeedCheck(speedTarget);
        timerCheck();

        if (inSpeedRange && robot.sorterHardware.openCheck() && waitingToFire) {
            fire();
        }

    }
}