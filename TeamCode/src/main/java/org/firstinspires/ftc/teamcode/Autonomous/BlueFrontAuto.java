package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Core.Robot.openClosed.CLOSED;
import static org.firstinspires.ftc.teamcode.Core.Robot.openClosed.OPEN;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Core.LauncherHardware;
import org.firstinspires.ftc.teamcode.Core.Robot;
import org.firstinspires.ftc.teamcode.Core.SorterHardware;
import org.firstinspires.ftc.teamcode.Vision.Limelight_Target_Scanner;
import org.firstinspires.ftc.teamcode.Vision.WaveTag;

import java.util.Objects;

@Autonomous(group = "Basic", name = "Blue Front Start")
public class BlueFrontAuto extends AutonomousPLUS {



    public static final String ALLIANCE_KEY = "Alliance";
    public static final String PATTERN_KEY = "Pattern";


    private Robot robot;

    public void runOpMode() {

        super.runOpMode();

        robot = new Robot(hardwareMap, telemetry, this);


        if(opModeInInit())
        {

            prepareAuto();

            robot.randomizationScanner.InitLimeLight(0, robot.hardwareMap);
            blackboard.put(ALLIANCE_KEY, "BLUE");
            while(opModeInInit())
            {
                robot.pattern = robot.randomizationScanner.GetRandomization();
                telemetry.addData(robot.pattern, " Works!");
                telemetry.update();

            }
        }
        robot.readyHardware(true);
        waitForStart();


        robot.sorterHardware.legalToSpin = true;

        //start with launcher facing goal, back of robot against goal
        robot.randomizationScanner.InitLimeLight(0, robot.hardwareMap);
        moveRobotForward(1000,12);
        turnRobotLeft(700,12);
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

            stallForSpin( robot.sorterHardware.fireSafeCheck(), robot.sorterHardware.positions[3]);
            robot.launcher.setLauncherSpeed(1);
            robot.targetTag = robot.targetScanner.tagInfo();
            turnRobotRight(700, 15);
            if (robot.targetTag.currentlyDetected) //Angle detect if possible / needed
            {
                turnRobotLeft((int) ((robot.targetTag.angleX +robot.limelightSideOffsetAngle) * (1660/360)), 1);
            }

            fireInSequence(robot.sorterHardware.positions[3],robot.sorterHardware.positions[5],robot.sorterHardware.positions[1]);
        }
        else if(robot.pattern.equals("PGP"))
        {


            stallForSpin(robot.sorterHardware.fireSafeCheck(), robot.sorterHardware.positions[3]);
            robot.launcher.setLauncherSpeed(1);
            robot.targetTag = robot.targetScanner.tagInfo();
            turnRobotRight(700, 15);
            if (robot.targetTag.currentlyDetected) //Angle detect if possible / needed
            {
                turnRobotRight((int) ((robot.targetTag.angleX +robot.limelightSideOffsetAngle) * (1660/360)), 1);
            }
            fireInSequence(robot.sorterHardware.positions[3], robot.sorterHardware.positions[1], robot.sorterHardware.positions[5]);
        }
        else if(robot.pattern.equals("GPP"))
        {
            stallForSpin(robot.sorterHardware.fireSafeCheck(), robot.sorterHardware.positions[1]);
            robot.launcher.setLauncherSpeed(1);
            robot.targetTag = robot.targetScanner.tagInfo();
            turnRobotRight(700, 15);
            if (robot.targetTag.currentlyDetected) //Angle detect if possible / needed
            {
                turnRobotLeft((int) ((robot.targetTag.angleX +robot.limelightSideOffsetAngle) * (1660/360)), 1);
            }
            fireInSequence(robot.sorterHardware.positions[1], robot.sorterHardware.positions[3], robot.sorterHardware.positions[5]);

        }
        else//Fire any*/
        {
            //robot.sorterHardware.prepareNewMovement(robot.sorterHardware.motor.getCurrentPosition(), robot.sorterHardware.positions[3]);
            stallForSpin(robot.sorterHardware.fireSafeCheck(), robot.sorterHardware.positions[1]);
            robot.launcher.setLauncherSpeed(1);
            robot.targetTag = robot.targetScanner.tagInfo();
            turnRobotRight(700, 15);
            if (robot.targetTag.currentlyDetected) //Angle detect if possible / needed
            {
                turnRobotLeft((int) ((robot.targetTag.angleX +robot.limelightSideOffsetAngle) * (1660/360)), 1);
            }

            fireInSequence(robot.sorterHardware.positions[1], robot.sorterHardware.positions[3], robot.sorterHardware.positions[5]);
        }
        speed = 1;
        //Unpark fully and line up with line of balls... may go wrong way


        turnRobotRight(450,12);
        moveRobotRight(450, 12);


        }





}