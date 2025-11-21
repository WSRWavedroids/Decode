package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Core.LauncherHardware;
import org.firstinspires.ftc.teamcode.Core.Robot;
import org.firstinspires.ftc.teamcode.Core.SorterHardware;
import org.firstinspires.ftc.teamcode.Vision.Limelight_Target_Scanner;
import org.firstinspires.ftc.teamcode.Vision.WaveTag;

import java.util.Objects;

@Autonomous(group = "Basic", name = "RED FRONT UNPARK")
public class redFrontUnpark extends AutonomousPLUS {

    public Limelight_Target_Scanner scanner;
    public String currentPosition;

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

        if (opModeInInit()) {

            //prepareAuto();

            while (opModeInInit()) {
                sleep(1);

            }
        }
        robot.readyHardware(true);
        blackboard.put(ALLIANCE_KEY, "RED");
        waitForStart();

        //start with launcher facing goal, back of robot against goal
        robot.randomizationScanner.InitLimeLight(0);
        moveRobotForward(1000, 5);
        turnRobotRight(600, 2);
        robot.pattern = robot.randomizationScanner.GetRandomization();
        moveRobotForward(250, 12);


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
        } else if (Objects.equals(blackboard.get(ALLIANCE_KEY), "RED")) {
            robot.targetScanner.InitLimeLightTargeting(1, robot);
            robot.scanningForTargetTag = true;
        } else {
            robot.targetScanner.InitLimeLightTargeting(1, robot);
            robot.scanningForTargetTag = true;
        }

    }


}

