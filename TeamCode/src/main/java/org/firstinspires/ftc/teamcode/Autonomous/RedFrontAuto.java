package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.Robot;

import java.util.Objects;

@Autonomous(group = "Basic", name = "RED FRONT SCORE")
public class RedFrontAuto extends AutonomousPLUS {

    private ElapsedTime stupidTimer;

    public static final String ALLIANCE_KEY = "Alliance";
    public static final String PATTERN_KEY = "Pattern";


    private Robot robot;

    public void runOpMode() {

        super.runOpMode();
        stupidTimer = new ElapsedTime();

        robot = new Robot(hardwareMap, telemetry, this);


        if(opModeInInit())
        {
            robot.readyHardware(true);
            robot.randomizationScanner.InitLimeLight(0);
            blackboard.put(ALLIANCE_KEY, "RED");
            telemetry.addData("Alliance set to", blackboard.get(ALLIANCE_KEY));
            while(opModeInInit())
            {
                telemetry.clear();
                robot.pattern = robot.randomizationScanner.GetRandomization();
                telemetry.addData("Auto is", " READY!");
                if(robot.pattern.equals("GGP"))
                {
                    telemetry.addData("None detected right now or...", robot.pattern);
                }
                else
                {
                    telemetry.addData("Detected", robot.pattern);
                }
                telemetry.update();

            }
        }

        waitForStart();


        robot.sorterHardware.legalToSpin = true;

        //start with launcher facing goal, back of robot against goal
        robot.randomizationScanner.InitLimeLight(0);
        moveRobotForward(1000,5);
        turnRobotRight(600,2);
        robot.pattern = robot.randomizationScanner.GetRandomization();


        telemetry.addData("Our pattern is: ", robot.pattern, " ...yay");

        switch (robot.pattern) {
            case "PPG":
                telemetry.addData("We doin", " PPG now");
                blackboard.put(PATTERN_KEY, "PPG");
                break;
            case "GPP":
                telemetry.addData("We doin", " GPP now");
                blackboard.put(PATTERN_KEY, "GPP");
                break;
            case "PGP":
                telemetry.addData("We doin", " PGP now");
                blackboard.put(PATTERN_KEY, "PGP");
                break;
            default:
                telemetry.addData("It failed ", "cry time");
                break;
        }
        telemetry.update();


        if (Objects.equals(blackboard.get(ALLIANCE_KEY), "BLUE")) {
            robot.targetScanner.InitLimeLightTargeting(2, robot);
            robot.scanningForTargetTag = true;
        } else if(Objects.equals(blackboard.get(ALLIANCE_KEY), "RED")) {
            robot.targetScanner.InitLimeLightTargeting(1, robot);
            robot.scanningForTargetTag = true;
        } else {
            robot.targetScanner.InitLimeLightTargeting(1, robot);
            robot.scanningForTargetTag = true;
        }

        switch (robot.pattern) {
            case "PPG":
                robot.launcher.setLauncherSpeed(1);
                robot.targetTag = robot.targetScanner.tagInfo();
                turnRobotLeft(550, 1);
                if (robot.targetTag.currentlyDetected) //Angle detect if possible / needed
                {
                    turnRobotRight((int) ((robot.targetTag.angleX + robot.limelightSideOffsetAngle) * (1660 / 360)), 1);
                }

                fireInSequence(robot.sorterHardware.positions[3], robot.sorterHardware.positions[5], robot.sorterHardware.positions[1]);
                //goGrabAPurple();
                break;
            case "PGP":
                robot.launcher.setLauncherSpeed(1);
                robot.targetTag = robot.targetScanner.tagInfo();
                turnRobotLeft(550, 1);
                if (robot.targetTag.currentlyDetected) //Angle detect if possible / needed
                {
                    turnRobotRight((int) ((robot.targetTag.angleX + robot.limelightSideOffsetAngle) * (1660 / 360)), 12);
                }
                fireInSequence(robot.sorterHardware.positions[3], robot.sorterHardware.positions[1], robot.sorterHardware.positions[5]);
                //goGrabAPurple();

                break;
            case "GPP":
                robot.launcher.setLauncherSpeed(1);
                robot.targetTag = robot.targetScanner.tagInfo();
                turnRobotLeft(550, 1);
                if (robot.targetTag.currentlyDetected) //Angle detect if possible / needed
                {
                    turnRobotRight((int) ((robot.targetTag.angleX + robot.limelightSideOffsetAngle) * (1660 / 360)), 1);
                }
                fireInSequence(robot.sorterHardware.positions[1], robot.sorterHardware.positions[3], robot.sorterHardware.positions[5]);
                //goGrabAGreen();

                break;
            default:
//Fire any*/

                //robot.sorterHardware.prepareNewMovement(robot.sorterHardware.motor.getCurrentPosition(), robot.sorterHardware.positions[3]);
                stallForSpin(robot.sorterHardware.positionedCheck(), robot.sorterHardware.positions[1]);
                robot.launcher.setLauncherSpeed(1);
                robot.targetTag = robot.targetScanner.tagInfo();
                turnRobotLeft(600, 15);
                if (robot.targetTag.currentlyDetected) //Angle detect if possible / needed
                {
                    turnRobotRight((int) ((robot.targetTag.angleX + robot.limelightSideOffsetAngle) * (1660 / 360)), 1);
                }

                fireInSequence(robot.sorterHardware.positions[1], robot.sorterHardware.positions[3], robot.sorterHardware.positions[5]);
                //goGrabAGreen();
                break;
        }
        speed = 1;
        //Unpark fully and line up with line of balls... may go wrong way

        turnRobotLeft(-1200,5);
        moveRobotRight(800, 5);
        moveRobotForward(200, 5);

    }

    void goGrabAGreen()
    {
        moveRobotRight(2400, 12);
        speed = 0.25;
        stallForSpin(robot.sorterHardware.positionedCheck(), robot.sorterHardware.positions[0]);
        robot.runAutoIntakeSequence();
        moveRobotForward(300, 12);
        robot.cancelAutoIntake();
        stallForSpin(robot.sorterHardware.positionedCheck(), robot.sorterHardware.positions[1]);
        speed = 1;
        moveRobotBackward(300, 12);
        moveRobotRight(2400, 12);
        turnRobotRight(-1200,12);
        fireOne(robot.sorterHardware.positions[1]);
        turnRobotLeft(-1200,12);
        moveRobotLeft(800, 12);
        moveRobotForward(200, 12);

    }

    void goGrabAPurple()
    {
        moveRobotRight(800, 12);
        speed = 0.25;
        stallForSpin(robot.sorterHardware.positionedCheck(), robot.sorterHardware.positions[0]);
        robot.runAutoIntakeSequence();
        moveRobotForward(300, 12);
        robot.cancelAutoIntake();
        stallForSpin(robot.sorterHardware.positionedCheck(), robot.sorterHardware.positions[1]);
        speed = 1;
        moveRobotBackward(300, 12);
        moveRobotLeft(800, 12);
        turnRobotRight(-1200,12);
        fireOne(robot.sorterHardware.positions[1]);
        turnRobotLeft(-1200,12);
        moveRobotLeft(800, 12);
        moveRobotForward(200, 12);
    }



    public void stallTillTrue(boolean condition)
    {
        while(!condition)
        {
            robot.updateAllDaThings();
            robot.sorterHardware.moveDoor();
            robot.sorterHardware.updateSorterHardware();
            robot.sorterHardware.runPIDMotorStuffLol();
            robot.launcher.updateLauncherHardware();
            robot.launcher.timerCheck();

            if(condition)
            {
                break;
            }
        }
    }

    void stallForSpin(boolean condition, int ticks)
    {
        int shortTermRef = robot.sorterHardware.findFastestRotationInTicks(robot.sorterHardware.motor.getCurrentPosition(), ticks);
        robot.sorterHardware.reference = shortTermRef;
        while(!condition)
        {

            robot.sorterHardware.reference  = robot.sorterHardware.findFastestRotationInTicks(robot.sorterHardware.motor.getCurrentPosition(), ticks);
            robot.updateAllDaThings();

            robot.sorterHardware.reference = shortTermRef;
            robot.sorterHardware.updateSorterHardware();
            robot.sorterHardware.runPIDMotorStuffLol();
            robot.launcher.updateLauncherHardware();
            robot.launcher.timerCheck();
            robot.launcher.runHammer();
            telemetry.addData("Spinning...", "Or stuck in loop :(");
            telemetry.addData("we in?", robot.sorterHardware.positionedCheck());
            if(robot.sorterHardware.positionedCheck())
            {
                break;
            }
            telemetry.update();
        }
    }

    void stallForTime(double time)
    {
        stupidTimer.reset();
        while(stupidTimer.seconds() < time)
        {
            robot.updateAllDaThings();
            robot.sorterHardware.moveDoor();
            robot.sorterHardware.updateSorterHardware();
            robot.sorterHardware.runPIDMotorStuffLol();

            robot.launcher.updateLauncherHardware();
            robot.launcher.timerCheck();
            //robot.launcher.runHammer();
            {

            }
        }
    }

    void stallForCondition(boolean condition)
    {
        while(!condition)
        {
            robot.updateAllDaThings();
            robot.sorterHardware.moveDoor();
            robot.sorterHardware.updateSorterHardware();
            robot.sorterHardware.runPIDMotorStuffLol();
            robot.launcher.updateLauncherHardware();
            robot.launcher.timerCheck();
            robot.launcher.runHammer();

            if(condition)
            {
                break;
            }
        }
    }


    public void fireInSequence(int one, int two, int three)
    {

        robot.launcher.setLauncherSpeed(1);
        //robot.sorterHardware.reference = robot.sorterHardware.findFastestRotationInTicks(robot.sorterHardware.motor.getCurrentPosition(), one);
        trySpammingSpin(false, one, 10);
        robot.doorServo.setPosition(robot.sorterHardware.doorOpenPosition);
        sleep(500);
        robot.doorServo.setPosition(robot.sorterHardware.doorClosedPosition);
        stallForTime(0.5);


        robot.launcher.setLauncherSpeed(1);
        //robot.sorterHardware.reference = robot.sorterHardware.findFastestRotationInTicks(robot.sorterHardware.motor.getCurrentPosition(), two);
        trySpammingSpin(false, two, 10);
        robot.doorServo.setPosition(robot.sorterHardware.doorOpenPosition);
        sleep(500);
        robot.doorServo.setPosition(robot.sorterHardware.doorClosedPosition);
        stallForTime(0.5);


        robot.launcher.setLauncherSpeed(1);
        //robot.sorterHardware.reference = robot.sorterHardware.findFastestRotationInTicks(robot.sorterHardware.motor.getCurrentPosition(), three);
        trySpammingSpin(false, three, 10);
        robot.doorServo.setPosition(robot.sorterHardware.doorOpenPosition);
        sleep(500);
        robot.doorServo.setPosition(robot.sorterHardware.doorClosedPosition);
        stallForTime(0.75);

        //reset to safe
        robot.launcher.setLauncherSpeed(0);
        trySpammingSpin(false, 0, 2);
    }

    public void fireTwo(int one, int two, int doortime)
    {

        robot.launcher.setLauncherSpeed(1);
        trySpammingSpin(robot.sorterHardware.positionedCheck(), one, 10);
        robot.doorServo.setPosition(robot.sorterHardware.doorOpenPosition);
        sleep(doortime);
        robot.doorServo.setPosition(robot.sorterHardware.doorClosedPosition);
        stallForTime(0.5);


        robot.launcher.setLauncherSpeed(1);
        trySpammingSpin(robot.sorterHardware.positionedCheck(), two, 10);
        robot.doorServo.setPosition(robot.sorterHardware.doorOpenPosition);
        sleep(doortime);
        robot.doorServo.setPosition(robot.sorterHardware.doorClosedPosition);
        stallForTime(0.5);

        //reset to safe
        robot.launcher.setLauncherSpeed(0);
        trySpammingSpin(robot.sorterHardware.positionedCheck(), 0, 2);
    }

    public void fireOne(int one)
    {

        robot.launcher.setLauncherSpeed(1);
        trySpammingSpin(robot.sorterHardware.positionedCheck(), one, 10);
        robot.doorServo.setPosition(robot.sorterHardware.doorOpenPosition);
        sleep(500);
        robot.doorServo.setPosition(robot.sorterHardware.doorClosedPosition);
        stallForTime(0.5);

        //reset to safe
        robot.launcher.setLauncherSpeed(0);
        trySpammingSpin(robot.sorterHardware.positionedCheck(), 0, 2);
    }

    void trySpammingSpin(boolean condition, int target, int numberOfSpams)
    {

        for(int i = 0; i < numberOfSpams; i++)
        {
            stallForSpin(condition, target);
        }

    }
}

