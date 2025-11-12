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

@Autonomous(group = "Basic", name = "Blue Back Start")
public class BlueBackAuto extends AutonomousPLUS {

    public Limelight_Randomization_Scanner Limelight = new Limelight_Randomization_Scanner();
    public Limelight_Target_Scanner scanner = new Limelight_Target_Scanner();
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
        targetData = robot.targetTag;
        launcher = robot.launcher;
        sorter = robot.sorterHardware;



        if(opModeInInit())
        {
            prepareAuto();
            robot.readyHardware(true);
            Limelight.InitLimeLight(0, robot.hardwareMap);
            blackboard.put(ALLIANCE_KEY, "BLUE");
            while(opModeInInit())
            {
                pattern = Limelight.GetRandomization();
                telemetry.addData(pattern, " Works!");
                telemetry.update();

            }
        }

        waitForStart();
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
            telemetry.addData("It failed: ", "Cry Time");
        }
        telemetry.update();


        if (blackboard.get(ALLIANCE_KEY).equals("BLUE")) {
            scanner.InitLimeLightTargeting(2, robot.hardwareMap);
            robot.scanningForTargetTag = true;
        } else if (blackboard.get(ALLIANCE_KEY).equals("RED")) {
            scanner.InitLimeLightTargeting(1, robot.hardwareMap);
            robot.scanningForTargetTag = true;
        } else {
            scanner.InitLimeLightTargeting(1, robot.hardwareMap);
            robot.scanningForTargetTag = true;
        }

        sorter.legalToSpin = true;

        speed = 0.75;

        launcher.setLauncherSpeed(launcher.findSpeed(targetData.distanceZ));

        if(pattern.equals("PPG"))
        {
            sorter.prepareNewMovement(sorter.motor.getCurrentPosition(), sorter.positions[3]);
            launcher.setLauncherSpeed(1);
            moveXY(450, 450, -450, true);
            targetData = scanner.tagInfo();
            if (targetData.currentlyDetected)
            {
                turnRobotLeft((int) (targetData.angleX +robot.limelightSideOffsetAngle), 1);
            }
            fireInSequence(sorter.positions[3], sorter.positions[5], sorter.positions[1], true);
        }
        else if(pattern.equals("PGP"))
        {
            sorter.prepareNewMovement(sorter.motor.getCurrentPosition(), sorter.positions[3]);
            launcher.setLauncherSpeed(1);
            moveXY(450, 450, -450, true);
            targetData = scanner.tagInfo();
            if (targetData.currentlyDetected)
            {
                turnRobotLeft((int) ((targetData.angleX +robot.limelightSideOffsetAngle) * (-1660/360)), 1);
            }
            fireInSequence(sorter.positions[3], sorter.positions[1], sorter.positions[5], true);

        }
        else if(pattern.equals("GPP"))
        {
            sorter.prepareNewMovement(sorter.motor.getCurrentPosition(), sorter.positions[1]);
            launcher.setLauncherSpeed(1);
            moveXY(450, 450, -450, true);
            targetData = scanner.tagInfo();
            if (targetData.currentlyDetected)
            {
                turnRobotLeft((int) ((targetData.angleX +robot.limelightSideOffsetAngle) * (-1660/360)), 1);
            }
            fireInSequence(sorter.positions[1], sorter.positions[5], sorter.positions[3], true);

        }
        else//Fire any
        {
            sorter.prepareNewMovement(sorter.motor.getCurrentPosition(), sorter.positions[1]);
            launcher.setLauncherSpeed(1);
            moveXY(450, 450, -450, true);
            targetData = scanner.tagInfo();
            if (targetData.currentlyDetected)
            {
                turnRobotLeft((int) ((targetData.angleX +robot.limelightSideOffsetAngle) * (-1660/360)), 1);
            }
            fireInSequence(1, 3, 5, true);
        }

        speed = 1;
        //Celebration Spin
        turnRobotRight(1000000000, 1);

        sleep(1000000000);

    }

    public void fireInSequence(int one, int two, int three, boolean skipOne)
    {
        if(!skipOne)
        {
            stallTillTrue(robot.sorterHardware.moveSafeCheck()); //Check to see if safe to spin, then do so
            launcher.setLauncherSpeed(launcher.findSpeed(targetData.distanceZ)); //set Motor target speed
            sorter.prepareNewMovement(sorter.motor.getCurrentPosition(), one); //command movement
            stallTillTrue(robot.sorterHardware.positionedCheck());
            stallTillTrue(launcher.inSpeedRange);// If we arent at speed yet, stall till we are
            sorter.triggerServo(OPEN);
            stallTillTrue(sorter.openCheck()); // Prepare to fire
            launcher.fire();
            stallTillTrue(!robot.launcher.waitingForServo); //Wait till done firing
            //launcher.setLauncherSpeed(0);// Cut laucher to save power
            sorter.triggerServo(CLOSED); //Tell to close
            stallTillTrue(sorter.closedCheck()); //Wait for close
        }



        stallTillTrue(robot.sorterHardware.moveSafeCheck()); //Check to see if safe to spin, then do so
        launcher.setLauncherSpeed(launcher.findSpeed(targetData.distanceZ)); //set Motor target speed
        sorter.prepareNewMovement(sorter.motor.getCurrentPosition(), two); //command movement
        stallTillTrue(robot.sorterHardware.positionedCheck());
        stallTillTrue(launcher.inSpeedRange);// If we arent at speed yet, stall till we are
        sorter.triggerServo(OPEN);
        stallTillTrue(sorter.openCheck()); // Prepare to fire
        launcher.fire();
        stallTillTrue(!robot.launcher.waitingForServo); //Wait till done firing
        //launcher.setLauncherSpeed(0);// Cut laucher to save power
        sorter.triggerServo(CLOSED); //Tell to close
        stallTillTrue(sorter.closedCheck()); //Wait for close



        stallTillTrue(robot.sorterHardware.moveSafeCheck()); //Check to see if safe to spin, then do so
        launcher.setLauncherSpeed(launcher.findSpeed(targetData.distanceZ)); //set Motor target speed
        sorter.prepareNewMovement(sorter.motor.getCurrentPosition(), three); //command movement
        stallTillTrue(robot.sorterHardware.positionedCheck());
        stallTillTrue(launcher.inSpeedRange);// If we arent at speed yet, stall till we are
        sorter.triggerServo(OPEN);
        stallTillTrue(sorter.openCheck()); // Prepare to fire
        launcher.fire();
        stallTillTrue(!robot.launcher.waitingForServo); //Wait till done firing
        //launcher.setLauncherSpeed(0);// Cut laucher to save power
        sorter.triggerServo(CLOSED); //Tell to close
        stallTillTrue(sorter.closedCheck()); //Wait for close
        launcher.setLauncherSpeed(0);
    }
}