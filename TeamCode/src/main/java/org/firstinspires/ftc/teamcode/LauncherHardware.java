package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class LauncherHardware {

    public boolean inSpeedRange;

    public Basic_Strafer_Bot disBot;
    public DcMotorEx motor;

    public double distanceMultiplier;

    public double P;
    public double I;
    public double D;
    public double F;

    public void initLaucher(Basic_Strafer_Bot robot)
    {
        disBot = robot;
        motor = disBot.launcherMotor;
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
    }

    public boolean motorSpeedCheck(DcMotorEx motor, double speedTarget, double tolleranceRange)
    {
        if((motor.getVelocity() > speedTarget-tolleranceRange) && (motor.getVelocity() < speedTarget+tolleranceRange))
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

    //public double speed












}