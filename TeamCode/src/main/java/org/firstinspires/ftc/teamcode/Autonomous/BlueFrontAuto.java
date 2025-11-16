package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Core.Robot.openClosed.CLOSED;
import static org.firstinspires.ftc.teamcode.Core.Robot.openClosed.OPEN;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Core.LauncherHardware;
import org.firstinspires.ftc.teamcode.Core.Robot;
import org.firstinspires.ftc.teamcode.Core.SorterHardware;
import org.firstinspires.ftc.teamcode.Vision.Limelight_Target_Scanner;
import org.firstinspires.ftc.teamcode.Vision.WaveTag;
import org.firstinspires.ftc.teamcode.Vision.Limelight_Randomization_Scanner;

import java.util.Objects;

@Autonomous(group = "Basic", name = "Blue Front Start")
public class BlueFrontAuto extends AutonomousPLUS {

    public Limelight_Target_Scanner scanner;
    public String currentPosition;
    public String pattern;

    public static final String ALLIANCE_KEY = "Alliance";
    public static final String PATTERN_KEY = "Pattern";

    public WaveTag targetData = null;
    public LauncherHardware launcher;
    public SorterHardware sorter;

    private Robot robot;

    public void runOpMode() {

        super.runOpMode();

        robot = new Robot(hardwareMap, telemetry, this);
        targetData = robot.targetTag; // comment out if error
        launcher = robot.launcher;
        sorter = robot.sorterHardware;
        robot.randomizationScanner = new Limelight_Randomization_Scanner();
        robot.targetScanner = new Limelight_Target_Scanner();

        randomization = robot.randomizationScanner;
        scanner = robot.targetScanner;



        if(opModeInInit())
        {

            prepareAuto();

            randomization.InitLimeLight(0, robot.hardwareMap);
            blackboard.put(ALLIANCE_KEY, "BLUE");
            while(opModeInInit())
            {
                pattern = randomization.GetRandomization();
                telemetry.addData(pattern, " Works!");
                telemetry.update();

            }
        }
        robot.readyHardware(true);
        waitForStart();


        robot.sorterHardware.legalToSpin = true;

        //start with launcher facing goal, back of robot against goal
        randomization.InitLimeLight(0, robot.hardwareMap);
        moveRobotForward(1000,12);
        turnRobotLeft(700,12);
        pattern = randomization.GetRandomization();
        robot.sorterHardware.legalToSpin = true;
        sorter.legalToSpin = true;

        telemetry.addData("Our pattern is: ", pattern, " ...yay");

        if(pattern.equals("PPG"))
        {
            telemetry.addData("We doin", " PPG now");
            blackboard.put(PATTERN_KEY, "PPG");
        }
        else if(pattern.equals("GPP"))
        {
            telemetry.addData("We doin", " GPP now");
            blackboard.put(PATTERN_KEY, "GPP");
        } else if (pattern.equals("PGP"))
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

        if(pattern.equals("PPG"))
        {

            stallTillready( robot.sorterHardware.positionedCheck(), 1365);
            robot.launcher.setLauncherSpeed(1);
            targetData = robot.targetScanner.tagInfo();
            turnRobotRight(700, 15);
            if (targetData.currentlyDetected) //Angle detect if possible / needed
            {
                turnRobotLeft((int) ((targetData.angleX +robot.limelightSideOffsetAngle) * (1660/360)), 1);
            }

            fireInSequence(1365,6827,4096, true);
        }
        else if(pattern.equals("PGP"))
        {


            stallTillready(robot.sorterHardware.positionedCheck(), 1365);
            robot.launcher.setLauncherSpeed(1);
            targetData = robot.targetScanner.tagInfo();
            turnRobotRight(700, 15);
            if (targetData.currentlyDetected) //Angle detect if possible / needed
            {
                turnRobotLeft((int) ((targetData.angleX +robot.limelightSideOffsetAngle) * (1660/360)), 1);
            }
            fireInSequence(1365, 4096, 6827, true);
        }
        else if(pattern.equals("GPP"))
        {
            stallTillready(robot.sorterHardware.positionedCheck(), 4096);
            robot.launcher.setLauncherSpeed(1);
            targetData = robot.targetScanner.tagInfo();
            turnRobotRight(700, 15);
            if (targetData.currentlyDetected) //Angle detect if possible / needed
            {
                turnRobotLeft((int) ((targetData.angleX +robot.limelightSideOffsetAngle) * (1660/360)), 1);
            }
            fireInSequence(4096, 1365, 6827, true);

        }
        else//Fire any*/
        {
            //robot.sorterHardware.prepareNewMovement(robot.sorterHardware.motor.getCurrentPosition(), robot.sorterHardware.positions[3]);
            stallTillready(robot.sorterHardware.positionedCheck(), 1365);
            robot.launcher.setLauncherSpeed(1);
            targetData = robot.targetScanner.tagInfo();
            turnRobotRight(700, 15);
            if (targetData.currentlyDetected) //Angle detect if possible / needed
            {
                turnRobotLeft((int) ((targetData.angleX +robot.limelightSideOffsetAngle) * (1660/360)), 1);
            }

            fireInSequence(4096, 1365, 6827, true);
        }
        speed = 1;
        //Unpark fully and line up with line of balls... may go wrong way
        stallTillready(robot.sorterHardware.positionedCheck(), 0);
        turnRobotRight(100,12);
        moveRobotRight(1000, 12);


        }

    public void fireInSequence(int one, int two, int three, boolean skipOne)
    {

        stallTillready(robot.sorterHardware.positionedCheck(), one);
        robot.launcher.setLauncherSpeed(1);
        robot.launcher.readyFire(1, true);
        stallTillready(!robot.launcher.onCooldown && !robot.sorterHardware.onCooldown); //Wait till done firing
        stallTillready(robot.sorterHardware.closedCheck()); //Wait for close



        stallTillready(robot.sorterHardware.positionedCheck(), two);
        robot.launcher.setLauncherSpeed(1);
        robot.launcher.readyFire(1, true);
        stallTillready(!robot.launcher.onCooldown && !sorter.onCooldown); //Wait till done firing
        stallTillready(sorter.closedCheck());


        stallTillready(robot.sorterHardware.positionedCheck(), three);
        robot.launcher.setLauncherSpeed(1);

        robot.launcher.readyFire(1, true);
        stallTillready(!robot.launcher.onCooldown && !robot.sorterHardware.onCooldown); //Wait till done firing
        stallTillready(robot.sorterHardware.closedCheck());

        robot.launcher.setLauncherSpeed(0);
    }

    void stallTillready(boolean condition, int ticks)
    {
        robot.sorterHardware.reference  = ticks;
        while(!condition)
        {
            robot.sorterHardware.reference  = ticks;
            robot.launcher.timerCheck();
            robot.launcher.updateLauncherHardware();
            robot.sorterHardware.runPIDMotorStuffLol(); // if this don't work all is lost
            robot.updateAllDaThings();
        }
    }

    void stallTillready(boolean condition)
    {
        while(!condition)
        {

            robot.launcher.timerCheck();
            robot.launcher.updateLauncherHardware();
            robot.sorterHardware.updateSorterHardware();
            robot.sorterHardware.runPIDMotorStuffLol();
            robot.updateAllDaThings();
        }
    }

}