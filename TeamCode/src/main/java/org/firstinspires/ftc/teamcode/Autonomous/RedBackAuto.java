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

@Autonomous(group = "Basic", name = "Red Back Start")
public class RedBackAuto extends AutonomousPLUS {


    public static final String ALLIANCE_KEY = "Alliance";
    public static final String PATTERN_KEY = "Pattern";


    private Robot robot;

    public void runOpMode() {

        super.runOpMode();

        robot = new Robot(hardwareMap, telemetry, this);



        if(opModeInInit())
        {
            prepareAuto();
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
            telemetry.addData("It failed: ", "Cry Time");
        }
        telemetry.update();


        if (blackboard.get(ALLIANCE_KEY).equals("BLUE")) {
            robot.targetScanner.InitLimeLightTargeting(2, robot.hardwareMap);
            robot.scanningForTargetTag = true;
        } else if (blackboard.get(ALLIANCE_KEY).equals("RED")) {
            robot.targetScanner.InitLimeLightTargeting(1, robot.hardwareMap);
            robot.scanningForTargetTag = true;
        } else {
            robot.targetScanner.InitLimeLightTargeting(1, robot.hardwareMap);
            robot.scanningForTargetTag = true;
        }

        robot.sorterHardware.legalToSpin = true;

        speed = 0.75;

        robot.launcher.setLauncherSpeed(robot.launcher.findSpeed(robot.targetTag.distanceZ));
        moveRobotForward(200, 2);
        moveRobotRight(200, 2);


        if(robot.launcher.equals("PPG"))
        {
            robot.sorterHardware.prepareNewMovement(robot.sorterHardware.motor.getCurrentPosition(), robot.sorterHardware.positions[3]);
            robot.launcher.setLauncherSpeed(1);
            robot.targetTag = robot.targetScanner.tagInfo();
            if (robot.targetTag.currentlyDetected)
            {
                turnRobotLeft((int) (robot.targetTag.angleX +robot.limelightSideOffsetAngle), 1);
            }
            fireInSequence(robot.sorterHardware.positions[3], robot.sorterHardware.positions[5], robot.sorterHardware.positions[1]);
        }
        else if(robot.pattern.equals("PGP"))
        {
            robot.sorterHardware.prepareNewMovement(robot.sorterHardware.motor.getCurrentPosition(), robot.sorterHardware.positions[3]);
            robot.launcher.setLauncherSpeed(1);
            robot.targetTag = robot.targetScanner.tagInfo();
            if (robot.targetTag.currentlyDetected)
            {
                turnRobotRight((int) ((robot.targetTag.angleX +robot.limelightSideOffsetAngle) * (-1660/360)), 1);
            }
            fireInSequence(robot.sorterHardware.positions[3], robot.sorterHardware.positions[1], robot.sorterHardware.positions[5]);

        }
        else if(robot.pattern.equals("GPP"))
        {
            robot.sorterHardware.prepareNewMovement(robot.sorterHardware.motor.getCurrentPosition(), robot.sorterHardware.positions[1]);
            robot.launcher.setLauncherSpeed(1);
            robot.targetTag = robot.targetScanner.tagInfo();
            if (robot.targetTag.currentlyDetected)
            {
                turnRobotRight((int) ((robot.targetTag.angleX +robot.limelightSideOffsetAngle) * (-1660/360)), 1);
            }
            fireInSequence(robot.sorterHardware.positions[1], robot.sorterHardware.positions[5], robot.sorterHardware.positions[3]);

        }
        else//Fire any
        {
            robot.sorterHardware.prepareNewMovement(robot.sorterHardware.motor.getCurrentPosition(), robot.sorterHardware.positions[1]);
            robot.launcher.setLauncherSpeed(1);
            robot.targetTag = robot.targetScanner.tagInfo();
            if (robot.targetTag.currentlyDetected)
            {
                turnRobotRight((int) ((robot.targetTag.angleX +robot.limelightSideOffsetAngle) * (-1660/360)), 1);
            }
            fireInSequence(1, 3, 5);
        }

        speed = 1;
        moveRobotRight(900, 12);//unpark
        sleep(1000000000);

    }

}