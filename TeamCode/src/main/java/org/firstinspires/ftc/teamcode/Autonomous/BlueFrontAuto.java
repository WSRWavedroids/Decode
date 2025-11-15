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


        sorter.legalToSpin = true;

        //start with launcher facing goal, back of robot against goal
        randomization.InitLimeLight(0, robot.hardwareMap);
        moveRobotForward(1000,12);
        turnRobotLeft(950,12);
        pattern = randomization.GetRandomization();
        sleep(500);


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
            scanner.InitLimeLightTargeting(2, robot.hardwareMap);
            robot.scanningForTargetTag = true;
        } else if(Objects.equals(blackboard.get(ALLIANCE_KEY), "RED")) {
            scanner.InitLimeLightTargeting(1, robot.hardwareMap);
            robot.scanningForTargetTag = true;
        } else {
            scanner.InitLimeLightTargeting(1, robot.hardwareMap);
            robot.scanningForTargetTag = true;
        }

        if(pattern.equals("PPG"))
        {
            sorter.prepareNewMovement(sorter.motor.getCurrentPosition(), sorter.positions[3]);
            stallTillTrue(sorter.positionedCheck());
            launcher.setLauncherSpeed(1);
            targetData = scanner.tagInfo();
            turnRobotRight(950, 15);
            if (targetData.currentlyDetected) //Angle detect if possible / needed
            {
                turnRobotLeft((int) ((targetData.angleX +robot.limelightSideOffsetAngle) * (1660/360)), 1);
            }

            fireInSequence(sorter.positions[3], sorter.positions[5], sorter.positions[1], true);
        }
        else if(pattern.equals("PGP"))
        {
            sorter.prepareNewMovement(sorter.motor.getCurrentPosition(), sorter.positions[3]);
            stallTillTrue(sorter.positionedCheck());
            launcher.setLauncherSpeed(1);
            turnRobotRight(950,1);
            targetData = scanner.tagInfo();
            if (targetData.currentlyDetected)
            {
                turnRobotRight((int) ((targetData.angleX +robot.limelightSideOffsetAngle) * (-1660/360)), 1);
            }
            fireInSequence(sorter.positions[3], sorter.positions[1], sorter.positions[5], true);

        }
        else if(pattern.equals("GPP"))
        {
            sorter.prepareNewMovement(sorter.motor.getCurrentPosition(), sorter.positions[1]);
            stallTillTrue(sorter.positionedCheck());
            launcher.setLauncherSpeed(1);
            turnRobotRight(950,1);
            targetData = scanner.tagInfo();
            if (targetData.currentlyDetected)
            {
                turnRobotRight((int) ((targetData.angleX +robot.limelightSideOffsetAngle) * (-1660/360)), 1);
            }
            fireInSequence(sorter.positions[1], sorter.positions[3], sorter.positions[5], true);

        }
        else//Fire any*/
        {

            sorter.prepareNewMovement(sorter.motor.getCurrentPosition(), sorter.positions[0]);
            stallTillTrue(sorter.positionedCheck());
            launcher.setLauncherSpeed(1);
            turnRobotRight(900,1);
            targetData = scanner.tagInfo();
            if (targetData.currentlyDetected)
            {
                turnRobotRight((int) ((targetData.angleX +robot.limelightSideOffsetAngle) * (-1660/360)), 1);
            }

            fireInSequence(sorter.positions[1], sorter.positions[3], sorter.positions[5], true);
        }
        speed = 1;
        //Unpark fully and line up with line of balls... may go wrong way
        sorter.prepareNewMovement(sorter.motor.getCurrentPosition(), sorter.positions[0]);
        moveRobotRight(1000, 12);


        }

    public void fireInSequence(int one, int two, int three, boolean skipOne)
    {
        if(!skipOne)
        {
            sorter.prepareNewMovement(sorter.motor.getCurrentPosition(), one); //command movement
            stallTillTrue(sorter.fireSafeCheck());
            prepareNextAction(500);// If we arent at speed yet, stall till we are
            launcher.readyFire(1);
            stallTillTrue(!robot.launcher.onCooldown && !sorter.onCooldown); //Wait till done firing
            stallTillTrue(sorter.closedCheck()); //Wait for close
        }
        else
        {
            stallTillTrue(sorter.fireSafeCheck());// If we arent at speed yet, stall till we are
            launcher.readyFire(1);
            stallTillTrue(!robot.launcher.onCooldown && !sorter.onCooldown); //Wait till done firing
            stallTillTrue(sorter.closedCheck());
        }


        sorter.prepareNewMovement(sorter.motor.getCurrentPosition(), two); //command movement
        stallTillTrue(sorter.fireSafeCheck());// If we arent at speed yet, stall till we are
        prepareNextAction(500);
        launcher.readyFire(1);
        stallTillTrue(!robot.launcher.onCooldown && !sorter.onCooldown); //Wait till done firing
        stallTillTrue(sorter.closedCheck());


        sorter.prepareNewMovement(sorter.motor.getCurrentPosition(), three); //command movement
        stallTillTrue(sorter.fireSafeCheck());// If we arent at speed yet, stall till we are
        prepareNextAction(500);
        launcher.readyFire(1);
        stallTillTrue(!robot.launcher.onCooldown && !sorter.onCooldown); //Wait till done firing
        stallTillTrue(sorter.closedCheck());

        launcher.setLauncherSpeed(0);
    }

}