package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Core.Robot.openClosed.CLOSED;
import static org.firstinspires.ftc.teamcode.Core.Robot.openClosed.OPEN;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.LauncherHardware;
import org.firstinspires.ftc.teamcode.Core.Robot;
import org.firstinspires.ftc.teamcode.Core.SorterHardware;
import org.firstinspires.ftc.teamcode.Vision.Limelight_Target_Scanner;
import org.firstinspires.ftc.teamcode.Vision.WaveTag;

import java.util.Objects;

@Autonomous(group = "Basic", name = "Red Front Start")
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
            robot.randomizationScanner.InitLimeLight(0, robot.hardwareMap);
            blackboard.put(ALLIANCE_KEY, "RED");
            while(opModeInInit())
            {
                robot.pattern = robot.randomizationScanner.GetRandomization();
                telemetry.addData(robot.pattern, " Works!");
                telemetry.update();

            }
        }

        waitForStart();


        robot.sorterHardware.legalToSpin = true;

        //start with launcher facing goal, back of robot against goal
        robot.randomizationScanner.InitLimeLight(0, robot.hardwareMap);
        moveRobotForward(1000,12);
        turnRobotRight(600,12);
        robot.pattern = robot.randomizationScanner.GetRandomization();
        robot.sorterHardware.legalToSpin = true;

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
            robot.targetScanner.InitLimeLightTargeting(2, robot.hardwareMap);
            robot.scanningForTargetTag = true;
        } else if(Objects.equals(blackboard.get(ALLIANCE_KEY), "RED")) {
            robot.targetScanner.InitLimeLightTargeting(1, robot.hardwareMap);
            robot.scanningForTargetTag = true;
        } else {
            robot.targetScanner.InitLimeLightTargeting(1, robot.hardwareMap);
            robot.scanningForTargetTag = true;
        }

        if(robot.pattern.equals("PPG"))
        {

            stallForSpin( robot.sorterHardware.positionedCheck(), robot.sorterHardware.positions[3]);
            robot.launcher.setLauncherSpeed(1);
            robot.targetTag = robot.targetScanner.tagInfo();
            turnRobotLeft(600, 15);
            if (robot.targetTag.currentlyDetected) //Angle detect if possible / needed
            {
                turnRobotLeft((int) ((robot.targetTag.angleX +robot.limelightSideOffsetAngle) * (1660/360)), 1);
            }

            fireInSequence(robot.sorterHardware.positions[3],robot.sorterHardware.positions[5],robot.sorterHardware.positions[1]);
        }
        else if(robot.pattern.equals("PGP"))
        {


            stallForSpin(robot.sorterHardware.positionedCheck(), robot.sorterHardware.positions[3]);
            robot.launcher.setLauncherSpeed(1);
            robot.targetTag = robot.targetScanner.tagInfo();
            turnRobotLeft(600, 15);
            if (robot.targetTag.currentlyDetected) //Angle detect if possible / needed
            {
                turnRobotLeft((int) ((robot.targetTag.angleX +robot.limelightSideOffsetAngle) * (1660/360)), 12);
            }
            fireInSequence(robot.sorterHardware.positions[3], robot.sorterHardware.positions[1], robot.sorterHardware.positions[5]);

        }
        else if(robot.pattern.equals("GPP"))
        {
            stallForSpin(robot.sorterHardware.positionedCheck(), robot.sorterHardware.positions[1]);
            robot.launcher.setLauncherSpeed(1);
            robot.targetTag = robot.targetScanner.tagInfo();
            turnRobotLeft(600, 15);
            if (robot.targetTag.currentlyDetected) //Angle detect if possible / needed
            {
                turnRobotLeft((int) ((robot.targetTag.angleX +robot.limelightSideOffsetAngle) * (1660/360)), 1);
            }
            fireInSequence(robot.sorterHardware.positions[1], robot.sorterHardware.positions[3], robot.sorterHardware.positions[5]);

        }
        else//Fire any*/
        {
            //robot.sorterHardware.prepareNewMovement(robot.sorterHardware.motor.getCurrentPosition(), robot.sorterHardware.positions[3]);
            stallForSpin(robot.sorterHardware.positionedCheck(), robot.sorterHardware.positions[1]);
            robot.launcher.setLauncherSpeed(1);
            robot.targetTag = robot.targetScanner.tagInfo();
            turnRobotLeft(600, 15);
            if (robot.targetTag.currentlyDetected) //Angle detect if possible / needed
            {
                turnRobotLeft((int) ((robot.targetTag.angleX +robot.limelightSideOffsetAngle) * (1660/360)), 1);
            }

            fireInSequence(robot.sorterHardware.positions[1], robot.sorterHardware.positions[3], robot.sorterHardware.positions[5]);
        }
        speed = 1;
        //Unpark fully and line up with line of balls... may go wrong way


        turnRobotLeft(-1200,12);
        moveRobotRight(800, 12);
        moveRobotForward(200, 12);


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
        }
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