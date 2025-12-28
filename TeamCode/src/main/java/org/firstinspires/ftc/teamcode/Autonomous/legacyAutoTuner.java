package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Autonomous.legacyAutoTuner.mode.PAUSE;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.panels.Panels;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Core.Robot;

@Configurable
@Autonomous(group = "Basic", name = "Legacy Auto Tuner")
public class legacyAutoTuner extends AutonomousPLUS {

    /// ///192.168.43.1:8001  IP Address of panels, type this into your browser when on robot wifi
    /// Opening panels will allow you to change the values effecting the built in motor PIDS
    /// REMEMBER! Changing variables in panels DOES NOT change the hard coded value after runtime
    /// In order to save your changes, type them into the script, replacing the values here
    ///
    /// To change movement types, press the bumpers to cycle through, and read the driver hub screen
    /// To pause and unpause, press square
    ///
    /// If your movements don't work as intended, please reference your autonomousPLUS file
    /// and change the movements in this script to match your autos
    ///
    /// Yes, I know this would be better as a state machine, but this way works with more robots / codebases
    /// This script was written by Clay Kramer, FTC 13206 🌊🤖, 2025

    private Robot robot;
    //modes
    public enum mode {FB, LR, TRN, DGNL, TRI, PAUSE};
    private mode savedMode;
    //Panels changeable values
    public static double p = 5;
    public static int tollerance;
    public static int movementDistance = 900;
    public static double modularSpeed = 0.5;
    public static long timeBetweenMovements = 200;

    public void runOpMode() {
        super.runOpMode();
        robot = new Robot(hardwareMap, telemetry, this);
        robot.panels = Panels.INSTANCE;
        mode current = mode.PAUSE;
        waitForStart();
        robot.encoderReset();
        while(opModeIsActive())
        {
            if(current == mode.FB)
            {
                calibrateDriveTrain(tollerance, p);
                speed = modularSpeed;
                moveRobotForward(movementDistance, timeBetweenMovements);
                calibrateDriveTrain(tollerance, p);
                speed = modularSpeed;
                moveRobotBackward(movementDistance, timeBetweenMovements);
                changeModes(current);
                if(gamepad1.square)
                {
                    current = PAUSE;
                }
            }
            else if(current == mode.LR)
            {
                calibrateDriveTrain(tollerance, p);
                speed = modularSpeed;
                moveRobotLeft(movementDistance, timeBetweenMovements);
                calibrateDriveTrain(tollerance, p);
                speed = modularSpeed;
                moveRobotRight(movementDistance, timeBetweenMovements);
                changeModes(current);
                if(gamepad1.square)
                {
                    current = PAUSE;
                }
            }
            else if(current == mode.TRN)
            {
                calibrateDriveTrain(tollerance, p);
                speed = modularSpeed;
                turnRobotLeft(movementDistance, timeBetweenMovements);
                calibrateDriveTrain(tollerance, p);
                speed = modularSpeed;
                turnRobotRight(movementDistance, timeBetweenMovements);
                changeModes(current);
                if(gamepad1.square)
                {
                    current = PAUSE;
                }
            }
            else if(current == mode.DGNL)
            {
                calibrateDriveTrain(tollerance, p);
                speed = modularSpeed;
                moveDiagonalLeft(movementDistance, timeBetweenMovements);
                calibrateDriveTrain(tollerance, p);
                speed = modularSpeed;
                moveDiagonalRight(movementDistance, timeBetweenMovements);
                changeModes(current);
                if(gamepad1.square)
                {
                    current = PAUSE;
                }
            }
            else if(current == mode.TRI)
            {
                calibrateDriveTrain(tollerance, p);
                speed = modularSpeed;
                moveDiagonalRight((int) Math.sqrt((2* (movementDistance^2))), timeBetweenMovements); // Do a little trig lol
                calibrateDriveTrain(tollerance, p);
                speed = modularSpeed;
                moveRobotBackward(movementDistance, timeBetweenMovements);
                calibrateDriveTrain(tollerance, p);
                speed = modularSpeed;
                moveRobotLeft(movementDistance, timeBetweenMovements);
                changeModes(current);
                if(gamepad1.square)
                {
                    current = PAUSE;
                }
            }
            if(current == mode.PAUSE)
            {
                telemetry.addData("Paused", savedMode);
                calibrateDriveTrain(tollerance, p);
                speed = modularSpeed;
                changeModes(current);
                if(gamepad1.square)
                {
                    current = PAUSE;
                }
            }
            telemetry.addData("Current Mode is:", savedMode);
            telemetry.update();
        }
    }

    void changeModes(mode current)
    {
        if (savedMode == mode.FB) {
            if (gamepad1.dpad_right) {
                current = mode.LR;
                savedMode = current;
            } else if (gamepad1.dpad_left) {
                current = mode.TRI;
                savedMode = current;
            }
        } else if (savedMode == mode.LR) {
            if (gamepad1.dpad_right) {
                current = mode.TRN;
                savedMode = current;
            } else if (gamepad1.dpad_left) {
                current = mode.FB;
                savedMode = current;
            }
        } else if (savedMode == mode.TRN) {
            if (gamepad1.dpad_right) {
                current = mode.DGNL;
                savedMode = current;
            } else if (gamepad1.dpad_left) {
                current = mode.LR;
                savedMode = current;
            }
        } else if (savedMode == mode.DGNL) {
            if (gamepad1.dpad_right) {
                current = mode.TRI;
                savedMode = current;
            } else if (gamepad1.dpad_left) {
                current = mode.TRN;
                savedMode = current;
            }
        } else if (savedMode == mode.TRI) {
            if (gamepad1.dpad_right) {
                current = mode.FB;
                savedMode = current;
            } else if (gamepad1.dpad_left) {
                current = mode.DGNL;
                savedMode = current;
            }
        }
    }
}