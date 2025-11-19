package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.Robot;

import java.util.Objects;

@Autonomous(group = "Basic", name = "Blue Front Start")
public class BlueFrontAuto extends AutonomousPLUS {

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
            blackboard.put(ALLIANCE_KEY, "BLUE");
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
        turnRobotLeft(600,2);
        robot.pattern = robot.randomizationScanner.GetRandomization();


        telemetry.addData("Our pattern is: ", robot.pattern, " ...yay");

        if(robot.pattern.equals("PPG"))
        {
            telemetry.addData("We doin", " PPG now");
            blackboard.put(PATTERN_KEY, "PPG");
        }
        else if(robot.pattern.equals("GPP"))
        {
            telemetry.addData("We doin", " GPP now");
            blackboard.put(PATTERN_KEY, "GPP");
        } else if (robot.pattern.equals("PGP"))
        {
            telemetry.addData("We doin", " PGP now");
            blackboard.put(PATTERN_KEY, "PGP");
        }
        else
        {
            telemetry.addData("It failed ", "cry time");
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

        if(robot.pattern.equals("PPG"))
        {
            robot.launcher.setLauncherSpeed(1);
            robot.targetTag = robot.targetScanner.tagInfo();
            turnRobotRight(600, 1);
            if (robot.targetTag.currentlyDetected) //Angle detect if possible / needed
            {
                turnRobotLeft((int) ((robot.targetTag.angleX +robot.limelightSideOffsetAngle) * (1660/360)), 1);
            }

            fireInSequence(robot.sorterHardware.positions[3],robot.sorterHardware.positions[5],robot.sorterHardware.positions[1]);
            goGrabAPurple();
        }
        else if(robot.pattern.equals("PGP"))
        {
            robot.launcher.setLauncherSpeed(1);
            robot.targetTag = robot.targetScanner.tagInfo();
            turnRobotRight(600, 1);
            if (robot.targetTag.currentlyDetected) //Angle detect if possible / needed
            {
                turnRobotRight((int) ((robot.targetTag.angleX +robot.limelightSideOffsetAngle) * (1660/360)), 12);
            }
            fireInSequence(robot.sorterHardware.positions[3], robot.sorterHardware.positions[1], robot.sorterHardware.positions[5]);
            goGrabAPurple();

        }
        else if(robot.pattern.equals("GPP"))
        {
            robot.launcher.setLauncherSpeed(1);
            robot.targetTag = robot.targetScanner.tagInfo();
            turnRobotRight(600, 1);
            if (robot.targetTag.currentlyDetected) //Angle detect if possible / needed
            {
                turnRobotRight((int) ((robot.targetTag.angleX +robot.limelightSideOffsetAngle) * (1660/360)), 1);
            }
            fireInSequence(robot.sorterHardware.positions[1], robot.sorterHardware.positions[3], robot.sorterHardware.positions[5]);
            goGrabAGreen();

        }
        else//Fire any*/
        {
            //robot.sorterHardware.prepareNewMovement(robot.sorterHardware.motor.getCurrentPosition(), robot.sorterHardware.positions[3]);
            stallForSpin(robot.sorterHardware.positionedCheck(), robot.sorterHardware.positions[1]);
            robot.launcher.setLauncherSpeed(1);
            robot.targetTag = robot.targetScanner.tagInfo();
            turnRobotRight(600, 15);
            if (robot.targetTag.currentlyDetected) //Angle detect if possible / needed
            {
                turnRobotLeft((int) ((robot.targetTag.angleX +robot.limelightSideOffsetAngle) * (1660/360)), 1);
            }

            fireInSequence(robot.sorterHardware.positions[1], robot.sorterHardware.positions[3], robot.sorterHardware.positions[5]);
            goGrabAGreen();
        }
        speed = 1;
        //Unpark fully and line up with line of balls... may go wrong way

        turnRobotRight(-1200,12);

        }

    void goGrabAGreen()
    {
        moveRobotLeft(2400, 12);
        speed = 0.25;
        stallForSpin(robot.sorterHardware.positionedCheck(), robot.sorterHardware.positions[0]);
        robot.runAutoIntakeSequence();
        moveRobotForward(300, 12);
        robot.cancelAutoIntake();
        stallForSpin(robot.sorterHardware.positionedCheck(), robot.sorterHardware.positions[1]);
        speed = 1;
        moveRobotBackward(300, 12);
        moveRobotRight(2400, 12);
        turnRobotLeft(-1200,12);
        fireOne(robot.sorterHardware.positions[1]);
        turnRobotRight(-1200,12);
        moveRobotRight(800, 12);
        moveRobotForward(200, 12);

    }

    void goGrabAPurple()
    {
        moveRobotLeft(800, 12);
        speed = 0.25;
        stallForSpin(robot.sorterHardware.positionedCheck(), robot.sorterHardware.positions[0]);
        robot.runAutoIntakeSequence();
        moveRobotForward(300, 12);
        robot.cancelAutoIntake();
        stallForSpin(robot.sorterHardware.positionedCheck(), robot.sorterHardware.positions[1]);
        speed = 1;
        moveRobotBackward(300, 12);
        moveRobotRight(800, 12);
        turnRobotLeft(-1200,12);
        fireOne(robot.sorterHardware.positions[1]);
        turnRobotRight(-1200,12);
        moveRobotRight(800, 12);
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
        stallForSpin(robot.sorterHardware.positionedCheck(), one);
        stallForSpin(robot.sorterHardware.positionedCheck(), one);
        stallForSpin(robot.sorterHardware.positionedCheck(), one);
        stallForSpin(robot.sorterHardware.positionedCheck(), one);
        stallForSpin(robot.sorterHardware.positionedCheck(), one);
        stallForSpin(robot.sorterHardware.positionedCheck(), one);
        stallForSpin(robot.sorterHardware.positionedCheck(), one);
        stallForSpin(robot.sorterHardware.positionedCheck(), one);
        stallForSpin(robot.sorterHardware.positionedCheck(), one);
        stallForSpin(robot.sorterHardware.positionedCheck(), one);
        robot.doorServo.setPosition(robot.sorterHardware.doorOpenPosition);
        sleep(500);
        robot.doorServo.setPosition(robot.sorterHardware.doorClosedPosition);
        stallForTime(0.5);


        robot.launcher.setLauncherSpeed(1);
        stallForSpin(robot.sorterHardware.positionedCheck(), two);
        stallForSpin(robot.sorterHardware.positionedCheck(), two);
        stallForSpin(robot.sorterHardware.positionedCheck(), two);
        stallForSpin(robot.sorterHardware.positionedCheck(), two);
        stallForSpin(robot.sorterHardware.positionedCheck(), two);
        stallForSpin(robot.sorterHardware.positionedCheck(), two);
        stallForSpin(robot.sorterHardware.positionedCheck(), two);
        stallForSpin(robot.sorterHardware.positionedCheck(), two);
        stallForSpin(robot.sorterHardware.positionedCheck(), two);
        stallForSpin(robot.sorterHardware.positionedCheck(), two);
        robot.doorServo.setPosition(robot.sorterHardware.doorOpenPosition);
        sleep(500);
        robot.doorServo.setPosition(robot.sorterHardware.doorClosedPosition);
        stallForTime(0.5);


        robot.launcher.setLauncherSpeed(1);
        stallForSpin(robot.sorterHardware.positionedCheck(), three);
        stallForSpin(robot.sorterHardware.positionedCheck(), three);
        stallForSpin(robot.sorterHardware.positionedCheck(), three);
        stallForSpin(robot.sorterHardware.positionedCheck(), three);
        stallForSpin(robot.sorterHardware.positionedCheck(), three);
        stallForSpin(robot.sorterHardware.positionedCheck(), three);
        stallForSpin(robot.sorterHardware.positionedCheck(), three);
        stallForSpin(robot.sorterHardware.positionedCheck(), three);
        stallForSpin(robot.sorterHardware.positionedCheck(), three);
        stallForSpin(robot.sorterHardware.positionedCheck(), three);
        robot.doorServo.setPosition(robot.sorterHardware.doorOpenPosition);
        sleep(500);
        robot.doorServo.setPosition(robot.sorterHardware.doorClosedPosition);
        stallForTime(0.5);

        //reset to safe
        robot.launcher.setLauncherSpeed(0);
        stallForSpin(robot.sorterHardware.positionedCheck(), robot.sorterHardware.positions[0]);
        stallForSpin(robot.sorterHardware.positionedCheck(), robot.sorterHardware.positions[0]);
    }

    public void fireInSequence(int one, int two)
    {

        robot.launcher.setLauncherSpeed(1);
        stallForSpin(robot.sorterHardware.positionedCheck(), one);
        stallForSpin(robot.sorterHardware.positionedCheck(), one);
        stallForSpin(robot.sorterHardware.positionedCheck(), one);
        stallForSpin(robot.sorterHardware.positionedCheck(), one);
        stallForSpin(robot.sorterHardware.positionedCheck(), one);
        stallForSpin(robot.sorterHardware.positionedCheck(), one);
        stallForSpin(robot.sorterHardware.positionedCheck(), one);
        stallForSpin(robot.sorterHardware.positionedCheck(), one);
        stallForSpin(robot.sorterHardware.positionedCheck(), one);
        stallForSpin(robot.sorterHardware.positionedCheck(), one);
        robot.doorServo.setPosition(robot.sorterHardware.doorOpenPosition);
        sleep(500);
        robot.doorServo.setPosition(robot.sorterHardware.doorClosedPosition);
        stallForTime(0.5);


        robot.launcher.setLauncherSpeed(1);
        stallForSpin(robot.sorterHardware.positionedCheck(), two);
        stallForSpin(robot.sorterHardware.positionedCheck(), two);
        stallForSpin(robot.sorterHardware.positionedCheck(), two);
        stallForSpin(robot.sorterHardware.positionedCheck(), two);
        stallForSpin(robot.sorterHardware.positionedCheck(), two);
        stallForSpin(robot.sorterHardware.positionedCheck(), two);
        stallForSpin(robot.sorterHardware.positionedCheck(), two);
        stallForSpin(robot.sorterHardware.positionedCheck(), two);
        stallForSpin(robot.sorterHardware.positionedCheck(), two);
        stallForSpin(robot.sorterHardware.positionedCheck(), two);
        robot.doorServo.setPosition(robot.sorterHardware.doorOpenPosition);
        sleep(500);
        robot.doorServo.setPosition(robot.sorterHardware.doorClosedPosition);
        stallForTime(0.5);

        //reset to safe
        robot.launcher.setLauncherSpeed(0);
        stallForSpin(robot.sorterHardware.positionedCheck(), robot.sorterHardware.positions[0]);
        stallForSpin(robot.sorterHardware.positionedCheck(), robot.sorterHardware.positions[0]);
    }

    public void fireOne(int one)
    {

        robot.launcher.setLauncherSpeed(1);
        stallForSpin(robot.sorterHardware.positionedCheck(), one);
        stallForSpin(robot.sorterHardware.positionedCheck(), one);
        stallForSpin(robot.sorterHardware.positionedCheck(), one);
        stallForSpin(robot.sorterHardware.positionedCheck(), one);
        stallForSpin(robot.sorterHardware.positionedCheck(), one);
        stallForSpin(robot.sorterHardware.positionedCheck(), one);
        stallForSpin(robot.sorterHardware.positionedCheck(), one);
        stallForSpin(robot.sorterHardware.positionedCheck(), one);
        stallForSpin(robot.sorterHardware.positionedCheck(), one);
        stallForSpin(robot.sorterHardware.positionedCheck(), one);
        robot.doorServo.setPosition(robot.sorterHardware.doorOpenPosition);
        sleep(500);
        robot.doorServo.setPosition(robot.sorterHardware.doorClosedPosition);
        stallForTime(0.5);

        //reset to safe
        robot.launcher.setLauncherSpeed(0);
        stallForSpin(robot.sorterHardware.positionedCheck(), robot.sorterHardware.positions[0]);
        stallForSpin(robot.sorterHardware.positionedCheck(), robot.sorterHardware.positions[0]);
    }
}

