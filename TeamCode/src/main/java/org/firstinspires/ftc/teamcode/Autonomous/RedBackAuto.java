package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Core.Robot;

@Autonomous(group = "Basic", name = "RED BACK START")
public class RedBackAuto extends AutonomousPLUS {


    public static final String ALLIANCE_KEY = "Alliance";
    public static final String PATTERN_KEY = "Pattern";


    private Robot robot;

    public void runOpMode() {

        super.runOpMode();

        robot = new Robot(hardwareMap, telemetry, this);



        if(opModeInInit())
        {
            //prepareAuto();
            robot.readyHardware(true);
            robot.randomizationScanner.InitLimeLight(0);
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
        moveRobotForward(250, 12);}
}