package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
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

    public double P;
    public double I;
    public double D;
    public double F;

    public LauncherHardware(Robot robotFile) {
        robot = robotFile;
        motor = disBot.launcherMotor;
        motor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
        //hammerServo = robot.hammerServo
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
    }

    public void runHammer() {
        boolean forward;
        boolean back;


    }


    public void rampSpeed(double targetspeed) {
        motor.setVelocity(targetspeed);
    }

    public void cutSpeed() {
        motor.setVelocity(0);
    }

    public void updateLauncherHardware() {
        inSpeedRange = motorSpeedCheck(speedTarget);

        if (inSpeedRange && robot.sorterHardware.openCheck() && waitingToFire) {
            fire();
        }

    }
}