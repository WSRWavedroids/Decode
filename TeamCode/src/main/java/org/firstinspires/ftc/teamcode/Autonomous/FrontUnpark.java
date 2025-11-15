package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Core.LauncherHardware;
import org.firstinspires.ftc.teamcode.Core.Robot;
import org.firstinspires.ftc.teamcode.Core.SorterHardware;
import org.firstinspires.ftc.teamcode.Vision.Limelight_Randomization_Scanner;
import org.firstinspires.ftc.teamcode.Vision.Limelight_Target_Scanner;
import org.firstinspires.ftc.teamcode.Vision.WaveTag;

import java.util.Objects;

@Autonomous(group = "Basic", name = "Unpark")
public class FrontUnpark extends AutonomousPLUS {

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

        if(opModeInInit())
        {

            prepareAuto();

            while(opModeInInit())
            {
                sleep(1);

            }
        }
        robot.readyHardware(true);

        waitForStart();

        moveRobotForward(1000,12);
        moveRobotLeft(900,100);

        }
}