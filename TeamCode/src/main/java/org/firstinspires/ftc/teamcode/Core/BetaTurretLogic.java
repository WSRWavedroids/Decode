package org.firstinspires.ftc.teamcode.Core;

import static org.firstinspires.ftc.teamcode.Core.BetaTurretLogic.swivelControllers.FINE;
import static org.firstinspires.ftc.teamcode.Core.BetaTurretLogic.swivelControllers.RAW;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.R;

@Configurable
public class BetaTurretLogic {
     Robot robot;
    PIDMotorGroup launcherMotors;
    public static double launcherP;
    public static double launcherI;
    public static double launcherD;
    public static double launcherF;
    DcMotorEx swivelMotor;
    public static double rawP;
    public static double rawI;
    public static double rawD;
    public static double rawF;
    public static double fineP;
    public static double fineI;
    public static double fineD;
    public static double fineF;
    public static double tolerance;
    ezPID launcherController;
    ezPID rawSwivelController;
    ezPID fineSwivelController;
    double turretDegreesFromTarget;
    double fineDegreeWindow = 60.0;
    double safeDegreeDistance = 270;
    public double manualOverridePositionInDegs = 0;
    public double manualOverrideSpeedInTicks = 72;
    double heightOffsetModifier;
    double distanceModifier;
    public boolean autoTrackingModeOn;
    public boolean launcherOn = false;
    public boolean readyToFire = false;
    enum swivelControllers {RAW, FINE}
    public swivelControllers lastUsedSwivelController;

    public double minimumDistance;
    public boolean canLaunch;

    public Follower follower;


    public BetaTurretLogic(Robot robot, Follower followerIN) {
        this.robot = robot;
        follower = followerIN;
        launcherMotors = new PIDMotorGroup(2, robot.launcherMotor1, robot.launcherMotor2);
        swivelMotor = robot.swivelMotor;
        launcherController = new ezPID(launcherMotors, 28, launcherP, launcherI, launcherD,
                launcherF, 1.0, tolerance, ezPID.movementType.SPEED);

        fineSwivelController = new ezPID(swivelMotor, 8192 , fineP, fineI, fineD,
                fineF, 1.0, tolerance, ezPID.movementType.POSITION);

        rawSwivelController = new ezPID(swivelMotor, 8192 , rawP, rawI, rawD,
                rawF, 1.0, tolerance, ezPID.movementType.POSITION);


    }

    public void runTurret()
    {
        updateTurretPositionXY();
        //updateAngle();
        turretDegreesFromTarget = updateAngle() - ticksToDegrees(swivelMotor.getCurrentPosition(), 8192);

        if(Math.abs(turretDegreesFromTarget) < fineDegreeWindow && autoTrackingModeOn)
        {
            if(lastUsedSwivelController == RAW)
            {
                fineSwivelController.grabInfoFromPID(rawSwivelController.shareInfo());
            }
            lastUsedSwivelController = swivelControllers.FINE;
            fineSwivelController.runCalledPID(runToSafeAngle(updateAngle()));
        }
        else if(Math.abs(turretDegreesFromTarget) > fineDegreeWindow && autoTrackingModeOn)
        {
            if(lastUsedSwivelController == FINE)
            {
                rawSwivelController.grabInfoFromPID(fineSwivelController.shareInfo());
            }
            lastUsedSwivelController = RAW;
            rawSwivelController.runCalledPID(runToSafeAngle(updateAngle()));
        }
        else
        {
            if(lastUsedSwivelController == RAW)
            {
                fineSwivelController.grabInfoFromPID(rawSwivelController.shareInfo());
            }
            lastUsedSwivelController = swivelControllers.FINE;
            fineSwivelController.runCalledPID(runToSafeAngle(manualOverridePositionInDegs));
        }

        if(launcherOn && autoTrackingModeOn)
        {
            launcherController.runCalledPID(findLauncherTargetSpeed(checkDistance()));
        }
        else
        {
            launcherController.runCalledPID(manualOverrideSpeedInTicks);
        }

        readyToFire = launcherController.withinTolerance && fineSwivelController.withinTolerance;
    }

    double checkDistance()
    {
        if(robot.targetTag.currentlyDetected)
        {
            return robot.targetTag.distanceZ;
        }
        else
        {

            if(robot.alliance.equals(Robot.allianceSides.BLUE))
            {
                //A^2 + B^2 = C^2
                return Math.sqrt(Math.pow(robot.turretPosition.x - 12.5, 2) + Math.pow((robot.turretPosition.y - 135.73), 2));
            }
            else if(robot.alliance.equals(Robot.allianceSides.RED))
            {
                return Math.sqrt(Math.pow(robot.turretPosition.x - 131.5, 2) + Math.pow((robot.turretPosition.y - 135.73), 2));
            }
            else
            {
                return 0;
            }
        }
    }

    double updateAngle()
    /// Returns angle the launcher needs to move IN DEGREES
    {

        if(robot.targetTag.currentlyDetected)
        {
            //turretDegreesFromTarget = Math.abs(robot.targetTag.angleX);
            return robot.targetTag.angleX;
        }
        else
        {
            if(robot.alliance.equals(Robot.allianceSides.BLUE))
            {
                //gets the raw angle without accounting for heading
                double rawAngle = Math.toDegrees(Math.atan2((robot.turretPosition.x -12.51), (robot.turretPosition.y -136.15)));
                //Corrects for heading then hands off to safeAngle logic
                //turretDegreesFromTarget = Math.abs(-90 + (rawAngle));
                return robot.robotHeading - 90 + (rawAngle);
            }
            else if(robot.alliance.equals(Robot.allianceSides.RED))
            {
                //gets the raw angle without accounting for heading
                double rawAngle = Math.toDegrees(Math.atan2((robot.turretPosition.x -131.699), (robot.turretPosition.y -135.73)));
                //Corrects for heading then hands off to safeAngle logic
                //turretDegreesFromTarget =  Math.abs(-90 + (rawAngle));
                return robot.robotHeading - 90 + (rawAngle);
            }
            return 0; //Do trig from pedro coords and then add/sub from heading
        }
    }

    double ticksToRPM(double ticksPerSecIn, double encoderResolution)
    {
        return (ticksPerSecIn / encoderResolution) * 60;
    }

    int degreesToTicks(double degreesIN, int encoderResolution)
    {
        return (int) ((encoderResolution/360) * degreesIN);
    }

    int ticksToDegrees(double ticksIN, int encoderResolution)
    {
        return (int) ((ticksIN / encoderResolution) * 360);
    }


    int runToSafeAngle(double intINDegs) {

        double finalTargetDeg;
        double currentPosDeg = ticksToDegrees(swivelMotor.getCurrentPosition(), 8192);


        // 1. If the input angle is unsafe, move to its safe coterminal
        if (intINDegs > safeDegreeDistance) {
            finalTargetDeg = intINDegs - 360;
        } else if (intINDegs < -safeDegreeDistance) {
            finalTargetDeg = intINDegs + 360;
        }
        // 2. If the input is safe, check if its coterminal is also safe
        else {
            double coterminal = (intINDegs > 0) ? intINDegs - 360 : intINDegs + 360;

            if (withinSafeZone(coterminal)) {
                // Take the shorter movement
                double distNormal = Math.abs(intINDegs - currentPosDeg);
                double distCoterminal = Math.abs(coterminal - currentPosDeg);

                finalTargetDeg = (distCoterminal < distNormal) ? coterminal : intINDegs;
            } else {
                // Only the input is safe
                finalTargetDeg = intINDegs;
            }
        }

        turretDegreesFromTarget = finalTargetDeg - currentPosDeg;
        return degreesToTicks(finalTargetDeg, 8192);
    }

    double findLauncherTargetSpeed(double distanceFromTarget)
    {
       return (distanceFromTarget + heightOffsetModifier) * distanceModifier;
    }

    boolean withinSafeZone(double DegsIn)
    {
        return (DegsIn < safeDegreeDistance && DegsIn > -safeDegreeDistance);
    }

    void updateTurretPositionXY()
    {
       robot.robotPosition.x = follower.getPose().getX();
       robot.robotPosition.y = follower.getPose().getY();
       robot.robotHeading = Math.toDegrees(follower.getHeading());

       double rotatedX = robot.turretPositionOffsetXInches * Math.cos(robot.robotHeading) - robot.turretPositionOffsetYInches * Math.sin(robot.robotHeading);
       double rotatedY = robot.turretPositionOffsetXInches * Math.sin(robot.robotHeading) + robot.turretPositionOffsetYInches * Math.cos(robot.robotHeading);

        robot.turretPosition.x = robot.robotPosition.x + rotatedX;
        robot.turretPosition.y = robot.robotPosition.y + rotatedY;
    }

    void calibratePositionFromTag()
    {
        //Angle from our robot heading to tag
        double testVal = ticksToDegrees(swivelMotor.getCurrentPosition(), 8192) + robot.targetTag.angleX;



        Vector2 goalPosition = new Vector2();
        if(robot.targetTag.currentlyDetected && fineSwivelController.withinTolerance) {
            if (robot.alliance.equals(Robot.allianceSides.BLUE))
            {
                goalPosition.x = 17;
                goalPosition.y = 132;
                //goalPosition.heading = 55;
            } else
            {
                goalPosition.x = 123;
                goalPosition.y = 132;
                //goalPosition.heading = 125;
            }


            //Do some trig
            //robot.robotPosition.x = ;
            //robot.robotPosition.y = ;
            //robot.robotHeading = ;
        }
    }


}









