package org.firstinspires.ftc.teamcode.Autonomous;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.panels.Panels;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Core.LauncherHardware;
import org.firstinspires.ftc.teamcode.Core.Robot;
import org.firstinspires.ftc.teamcode.Core.SorterHardware;
import org.firstinspires.ftc.teamcode.Vision.Limelight_Target_Scanner;
import org.firstinspires.ftc.teamcode.Vision.Limelight_Randomization_Scanner;
import org.firstinspires.ftc.teamcode.Vision.WaveTag;

@Configurable
@Autonomous(group = "Basic", name = "Legacy Tuner")
public class legacyAutoTuner extends AutonomousPLUS {

    public Limelight_Randomization_Scanner Limelight = new Limelight_Randomization_Scanner();
    public Limelight_Target_Scanner scanner = new Limelight_Target_Scanner();
    public String currentPosition;
    public String pattern;

    public static final String ALLIANCE_KEY = "Alliance";
    public static final String PATTERN_KEY = "Pattern";

    //static TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    public WaveTag targetData = null;
    public LauncherHardware launcher;
    public SorterHardware sorter;

    private Robot robot;

    public static double p;
    public static int tollerance;
    public static int movementDistance = 900;
    public enum mode {FB, LR, TRN, DGNL};
    public double modularSpeed = 0.5;

    public void runOpMode() {

        super.runOpMode();

        robot = new Robot(hardwareMap, telemetry, this);
        targetData = robot.targetTag;
        launcher = robot.launcher;
        sorter = robot.sorterHardware;

        if (opModeInInit()) {
            //prepareAuto();
            Limelight.InitLimeLight(0, robot.hardwareMap);
            blackboard.put(ALLIANCE_KEY, "BLUE");
            while (opModeInInit()) {
                pattern = Limelight.GetRandomization();
                telemetry.addData(pattern, " Works!");
                telemetry.update();
            }
        }

        robot.panels = Panels.INSTANCE;
        mode current = mode.FB;
        waitForStart();

        robot.encoderReset();

        //192.168.43.1:8001  IP Address of panels

        while(opModeIsActive())
        {

            if(current == mode.FB)
            {
                calibrateDriveTrain(tollerance, p);
                speed = modularSpeed;
                moveRobotForward(movementDistance, 12);
                prepareNextAction(500);
                calibrateDriveTrain(tollerance, p);
                speed = modularSpeed;
                moveRobotBackward(movementDistance, 12);

                if(gamepad1.dpad_right)
                {
                   current = mode. LR;
                }
                else if(gamepad1.dpad_left)
                {
                    current = mode. DGNL;
                }
            }

            else if(current == mode.LR)
            {
                calibrateDriveTrain(tollerance, p);
                speed = modularSpeed;
                moveRobotLeft(movementDistance, 12);
                prepareNextAction(12);
                calibrateDriveTrain(tollerance, p);
                speed = modularSpeed;
                moveRobotRight(movementDistance, 12);

                if(gamepad1.dpad_right)
                {
                    current = mode. TRN;
                }
                else if(gamepad1.dpad_left)
                {
                    current = mode. FB;
                }
            }

            else if(current == mode.TRN)
            {
                calibrateDriveTrain(tollerance, p);
                speed = modularSpeed;
                turnRobotLeft(movementDistance, 12);
                prepareNextAction(12);
                calibrateDriveTrain(tollerance, p);
                speed = modularSpeed;
                turnRobotRight(movementDistance, 12);

                if(gamepad1.dpad_right)
                {
                    current = mode. DGNL;
                }
                else if(gamepad1.dpad_left)
                {
                    current = mode.LR;
                }
            }

            else if(current == mode.DGNL)
            {
                calibrateDriveTrain(tollerance, p);
                speed = modularSpeed;
                moveDiagonalLeft(movementDistance, 12);
                prepareNextAction(12);
                calibrateDriveTrain(tollerance, p);
                speed = modularSpeed;
                moveDiagonalRight(movementDistance, 12);

                if(gamepad1.dpad_right)
                {
                    current = mode. FB;
                }
                else if(gamepad1.dpad_left)
                {
                    current = mode. TRN;
                }
            }


        }
    }
}