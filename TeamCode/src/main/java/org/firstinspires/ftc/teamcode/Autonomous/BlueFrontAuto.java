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

        waitForStart();


        sorter.legalToSpin = true;

        //start with launcher facing goal, back of robot against goal
        randomization.InitLimeLight(0, robot.hardwareMap);
        moveRobotForward(1000, 4);
        turnRobotLeft(900, 500);
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


        if (blackboard.get(ALLIANCE_KEY).equals("BLUE")) {
            scanner.InitLimeLightTargeting(2, robot.hardwareMap);
        } else if (blackboard.get(ALLIANCE_KEY).equals("RED")) {
            scanner.InitLimeLightTargeting(1, robot.hardwareMap);
        } else {
            scanner.InitLimeLightTargeting(1, robot.hardwareMap);
        }



        if(pattern.equals("PPG"))
        {
            sorter.prepareNewMovement(sorter.motor.getCurrentPosition(), sorter.positions[3]);
            launcher.setLauncherSpeed(1);

            turnRobotRight(900, 15);
            if (targetData.currentlyDetected) //Angle detect if possible / needed
            {
                turnRobotRight((int) (targetData.angleX * (1660/360)), 1);
            }



            fireInSequence(sorter.positions[3], sorter.positions[5], sorter.positions[1], true);
        }
        else if(pattern.equals("PGP"))
        {
            sorter.prepareNewMovement(sorter.motor.getCurrentPosition(), sorter.positions[3]);
            launcher.setLauncherSpeed(1);
            turnRobotRight(900,1);
            targetData = scanner.tagInfo();
            if (targetData.currentlyDetected)
            {
                turnRobotRight((int) (targetData.angleX * (-1660/360)), 1);
            }
            fireInSequence(sorter.positions[3], sorter.positions[1], sorter.positions[5], true);

        }
        else if(pattern.equals("GPP"))
        {
            sorter.prepareNewMovement(sorter.motor.getCurrentPosition(), sorter.positions[1]);
            launcher.setLauncherSpeed(1);
            turnRobotRight(900,1);
            targetData = scanner.tagInfo();
            if (targetData.currentlyDetected)
            {
                turnRobotRight((int) (targetData.angleX * (-1660/360)), 1);
            }
            fireInSequence(sorter.positions[1], sorter.positions[3], sorter.positions[5], true);

        }
        else//Fire any*/
        {
            sorter.prepareNewMovement(sorter.motor.getCurrentPosition(), sorter.positions[0]);
            launcher.setLauncherSpeed(1);
            turnRobotRight(900,1);
            targetData = scanner.tagInfo();
            if (targetData.currentlyDetected)
            {
                turnRobotRight((int) (targetData.angleX * (-1660/360)), 1);
            }
            fireInSequence(1, 3, 5, true);
        }
        speed = 1;
        //Celebration Spins lol
        turnRobotRight(10000000, 15);

        }

    public void fireInSequence(int one, int two, int three, boolean skipOne)
    {
        if(!skipOne)
        {
            stallTillTrue(robot.sorterHardware.moveSafeCheck()); //Check to see if safe to spin, then do so
            launcher.setLauncherSpeed(1); //set Motor target speed
            sorter.prepareNewMovement(sorter.motor.getCurrentPosition(), sorter.positions[one]); //command movement
            stallTillTrue(robot.sorterHardware.positionedCheck());
            stallTillTrue(launcher.inSpeedRange);// If we arent at speed yet, stall till we are
            sorter.triggerServo(OPEN);
            stallTillTrue(sorter.openCheck()); // Prepare to fire
            launcher.fire();
            stallTillTrue(!robot.launcher.waitingForServo); //Wait till done firing
            launcher.setLauncherSpeed(0);// Cut laucher to save power
            sorter.triggerServo(CLOSED); //Tell to close
            stallTillTrue(sorter.closedCheck()); //Wait for close
        }
        else
        {
            stallTillTrue(robot.sorterHardware.positionedCheck());
            stallTillTrue(launcher.inSpeedRange);// If we arent at speed yet, stall till we are
            sorter.triggerServo(OPEN);
            stallTillTrue(sorter.openCheck()); // Prepare to fire
            launcher.fire();
            stallTillTrue(!robot.launcher.waitingForServo); //Wait till done firing
            launcher.setLauncherSpeed(0);// Cut laucher to save power
            sorter.triggerServo(CLOSED); //Tell to close
            stallTillTrue(sorter.closedCheck());
        }



        stallTillTrue(robot.sorterHardware.moveSafeCheck()); //Check to see if safe to spin, then do so
        launcher.setLauncherSpeed(1); //set Motor target speed
        sorter.prepareNewMovement(sorter.motor.getCurrentPosition(), sorter.positions[two]); //command movement
        stallTillTrue(robot.sorterHardware.positionedCheck());
        stallTillTrue(launcher.inSpeedRange);// If we arent at speed yet, stall till we are
        sorter.triggerServo(OPEN);
        stallTillTrue(sorter.openCheck()); // Prepare to fire
        launcher.fire();
        stallTillTrue(!robot.launcher.waitingForServo); //Wait till done firing
        launcher.setLauncherSpeed(0);// Cut laucher to save power
        sorter.triggerServo(CLOSED); //Tell to close
        stallTillTrue(sorter.closedCheck()); //Wait for close



        stallTillTrue(robot.sorterHardware.moveSafeCheck()); //Check to see if safe to spin, then do so
        launcher.setLauncherSpeed(1); //set Motor target speed
        sorter.prepareNewMovement(sorter.motor.getCurrentPosition(), sorter.positions[three]); //command movement
        stallTillTrue(robot.sorterHardware.positionedCheck());
        stallTillTrue(launcher.inSpeedRange);// If we arent at speed yet, stall till we are
        sorter.triggerServo(OPEN);
        stallTillTrue(sorter.openCheck()); // Prepare to fire
        launcher.fire();
        stallTillTrue(!robot.launcher.waitingForServo); //Wait till done firing
        launcher.setLauncherSpeed(0);// Cut laucher to save power
        sorter.triggerServo(CLOSED); //Tell to close
        stallTillTrue(sorter.closedCheck()); //Wait for close
    }

}