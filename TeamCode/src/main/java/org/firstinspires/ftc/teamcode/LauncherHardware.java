package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class LauncherHardware {

    public boolean inSpeedRange;

    public Basic_Strafer_Bot disBot;
    public DcMotorEx motor;

    public boolean on = false;

    public double toleranceRange = 5;

    public double speedTarget;

    public double distanceMultiplier;

    public double P;
    public double I;
    public double D;
    public double F;

    public void initLaucher(Basic_Strafer_Bot robot)
    {
        disBot = robot;
        motor = disBot.launcherMotor;
        motor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
    }

    public boolean motorSpeedCheck(double speedTarget)
    {
        if((motor.getVelocity() > speedTarget-toleranceRange) && (motor.getVelocity() < speedTarget+toleranceRange))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    public double findSpeed(double distance)
    {
        return distance * distanceMultiplier;
    }

    public void rampSpeed(double targetspeed)
    {
        motor.setVelocity(targetspeed);
    }
    public void cutSpeed()
    {
        motor.setVelocity(0);
    }

    public void updateLauncherHardware()
    {
        inSpeedRange = motorSpeedCheck(speedTarget);


    }



    //public double speed












}