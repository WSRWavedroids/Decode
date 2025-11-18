package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Autonomous.BetaBlueFrontAuto.step.CHECK_MOVE_1;
import static org.firstinspires.ftc.teamcode.Autonomous.BetaBlueFrontAuto.step.CHECK_MOVE_2;
import static org.firstinspires.ftc.teamcode.Autonomous.BetaBlueFrontAuto.step.CHECK_TAG;
import static org.firstinspires.ftc.teamcode.Autonomous.BetaBlueFrontAuto.step.FINE_TUNE_TARGETING;
import static org.firstinspires.ftc.teamcode.Autonomous.BetaBlueFrontAuto.step.FIRE_FIRST_PATTERN;
import static org.firstinspires.ftc.teamcode.Autonomous.BetaBlueFrontAuto.step.FIRST_SPIN;
import static org.firstinspires.ftc.teamcode.Autonomous.BetaBlueFrontAuto.step.LAUNCHER_ON;
import static org.firstinspires.ftc.teamcode.Autonomous.BetaBlueFrontAuto.step.SET_APRILTAG_PIPELINE;
import static org.firstinspires.ftc.teamcode.Autonomous.BetaBlueFrontAuto.step.TAG_TELEMETRY;
import static org.firstinspires.ftc.teamcode.Autonomous.BetaBlueFrontAuto.step.TURN_BACK_TOWARDS_GOAL;
import static org.firstinspires.ftc.teamcode.Autonomous.BetaBlueFrontAuto.step.UNPARK_1;
import static org.firstinspires.ftc.teamcode.Autonomous.BetaBlueFrontAuto.step.UNPARK_2;
import static org.firstinspires.ftc.teamcode.Autonomous.BetaBlueFrontAuto.step.UNPARK_3;
import static org.firstinspires.ftc.teamcode.Autonomous.BetaBlueFrontAuto.step.YAY;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.Robot;

import java.util.Objects;

/**
 * This file is our iterative (Non-Linear) "OpMode" for TeleOp.
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is selected on the Robot Controller and executed.
 * This particular one is called "Lean Mean TeleOp Machine". I had a little too much fun with naming this.
 * <p>
 * This OpMode controls the functions of the robot during the driver-controlled period.
 * <p>
 * If the "@Disabled" line is not commented out, the program will not show up on the driver hub.
 * If you ever have problems with the program not showing up on the driver hub, it's probably because of that.
 * <p>
 * Throughout this program, there are comments explaining what everything does because previous programmers
 * did a horrible job of doing that.
 */
@Autonomous(group = "Basic", name = "Beta Blue Front Start")
public class BetaBlueFrontAuto extends OpMode {

    // This section tells the program all of the different pieces of hardware that are on our robot that we will use in the program.
    private ElapsedTime runtime = new ElapsedTime();
    private double speed = 0.75;

    int slot = 0; // temp for testing lol

    int targetOffset = 0;

    //private double storedSpeed;
    public Robot robot = null;
    public AutonomousPlusPLUS auto = null;
    public IMU imu;

    enum step {
        CHECK_MOVE_1, CHECK_MOVE_2, CHECK_TAG, TAG_TELEMETRY, SET_APRILTAG_PIPELINE,
        FIRST_SPIN, LAUNCHER_ON, TURN_BACK_TOWARDS_GOAL, FINE_TUNE_TARGETING, FIRE_FIRST_PATTERN,
        UNPARK_1, UNPARK_2, UNPARK_3,
        YAY

    }
    private step currentStep;


    public static final String ALLIANCE_KEY = "Alliance"; //For blackboard
    public static final String PATTERN_KEY = "Pattern";

    /*
     * Code to run ONCE when the driver hits INIT
     */
    public void init() {

        // Call the initialization protocol from the Robot class.
        robot = new Robot(hardwareMap, telemetry, this);
        auto = new AutonomousPlusPLUS(robot);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        robot.randomizationScanner.InitLimeLight(0, robot.hardwareMap);
        blackboard.put(ALLIANCE_KEY, "BLUE");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    public void init_loop() {
        telemetry.addData("HYPE", "ARE! YOU! READY?!?!?!?!");

        robot.pattern = robot.randomizationScanner.GetRandomization();
        telemetry.addData(robot.pattern, " Works!");
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    public void start() {
        runtime.reset();
        telemetry.addData("HYPE", "Let's do this!!!");
        gamepad1.setLedColor(0, 0, 255, 100000000);
        gamepad2.setLedColor(0, 0, 255, 100000000);
        robot.sorterHardware.resetSorterEncoder();//REMOVE ONCE AUTO -> TELE IS FIGURED OUT
        robot.sorterHardware.legalToSpin = true;

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    public void loop() {
        switch (currentStep) {
            case CHECK_MOVE_1:
                auto.moveRobotForward(1000);
                nextStep(CHECK_MOVE_2);
            case CHECK_MOVE_2:
                if (auto.checkMovement()) {
                    auto.turnRobotLeft(600);
                    nextStep(CHECK_TAG);
                }
            case CHECK_TAG:
                if (auto.checkMovement()) {
                    robot.pattern = robot.randomizationScanner.GetRandomization();
                    robot.sorterHardware.legalToSpin = true;
                    nextStep(TAG_TELEMETRY);
                }
            case TAG_TELEMETRY:
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
                    telemetry.addData("It failed ", "cry time");
                }

                nextStep(SET_APRILTAG_PIPELINE);
            case SET_APRILTAG_PIPELINE:
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

                nextStep(FIRST_SPIN);
            case FIRST_SPIN:
                if (robot.pattern.equals("PGP") || robot.pattern.equals("PPG")) {
                    robot.sorterHardware.prepareNewMovement(
                            robot.sorterHardware.motor.getCurrentPosition(),
                            robot.sorterLogic.slotB.getFirePosition());
                } else {
                    robot.sorterHardware.prepareNewMovement(
                            robot.sorterHardware.motor.getCurrentPosition(),
                            robot.sorterLogic.slotA.getFirePosition());
                }

                nextStep(LAUNCHER_ON);
            case LAUNCHER_ON:
                if (robot.sorterHardware.positionedCheck()) {
                    robot.launcher.setLauncherSpeed(1);
                    robot.targetTag = robot.targetScanner.tagInfo();
                    nextStep(TURN_BACK_TOWARDS_GOAL);
                }
            case TURN_BACK_TOWARDS_GOAL:
                auto.turnRobotRight(600);
                nextStep(FINE_TUNE_TARGETING);
            case FINE_TUNE_TARGETING:
                if (auto.checkMovement()) {
                    if (robot.targetTag.currentlyDetected) //Angle detect if possible / needed
                    {
                        auto.turnRobotRight((int) ((robot.targetTag.angleX +robot.limelightSideOffsetAngle) * (1660/360)));
                    }
                    nextStep(FIRE_FIRST_PATTERN);
                }
            case FIRE_FIRST_PATTERN:
                if (auto.checkMovement()) {
                    if (robot.pattern.equals("PPG")) {
                        auto.fireInSequence(robot.sorterHardware.positions[3],robot.sorterHardware.positions[5],robot.sorterHardware.positions[1]);
                    } else if (robot.pattern.equals("PGP")) {
                        auto.fireInSequence(robot.sorterHardware.positions[3], robot.sorterHardware.positions[1], robot.sorterHardware.positions[5]);
                    } else {
                        auto.fireInSequence(robot.sorterHardware.positions[1], robot.sorterHardware.positions[3], robot.sorterHardware.positions[5]);
                    }

                    nextStep(UNPARK_1);
                }
            case UNPARK_1:
                auto.turnRobotRight(-1200);
                nextStep(UNPARK_2);
            case UNPARK_2:
                if (auto.checkMovement()) {
                    auto.moveRobotLeft(800);
                    nextStep(UNPARK_3);
                }
            case UNPARK_3:
                if (auto.checkMovement()) {
                    auto.moveRobotForward(200);
                    nextStep(YAY);
                }
            case YAY:
                super.requestOpModeStop();
        }

        robot.updateAllDaThings();
        doTelemetryStuff();

    }

    /**
     * Code to run ONCE after the driver hits STOP
     */
    public void stop() {
        telemetry.addData("Status", "Robot Stopped");
    }


    /*
     * The holding cell for all of the random functions we call above.
     */
    private void nextStep(step nextStep) {
        currentStep = nextStep;
    }

    private void nextPatternSpecificStep(step nextGeneralStep) {

    }

    private void doTelemetryStuff() {
        // This little section updates the driver hub on the runtime and the motor powers.
        // It's mostly used for troubleshooting.
        telemetry.addData("Status", "Run Time: " + runtime.toString());


        if(robot.targetTag.currentlyDetected)
        {
            telemetry.addData("last detected x angle: ", robot.targetTag.angleX);
            telemetry.addData("last detected y angle: ", robot.targetTag.angleY);

            telemetry.addData("last distance x: ", robot.targetTag.distanceX);
            telemetry.addData("last detected distance y: ", robot.targetTag.distanceY);
            telemetry.addData("last detected distance z: ", robot.targetTag.distanceZ);
        }

        telemetry.addData("Last saved pattern: ", blackboard.get(PATTERN_KEY));

        telemetry.addData("Last saved Alliance: ", blackboard.get(ALLIANCE_KEY));

        telemetry.addData("Reference", robot.sorterHardware.reference);

        telemetry.addData("Blender in position", robot.sorterHardware.inProperTickPosition());
        telemetry.addData("Closed Check", robot.sorterHardware.closedCheck());
        telemetry.addData("Equalized Target Position", robot.sorterLogic.offsetPositions.get(targetOffset));
        telemetry.addData("Door Open", robot.sorterHardware.open);
        telemetry.addData("Door Target", robot.sorterHardware.doorTarget);
        telemetry.addData("Launcher Velocity", robot.launcher.motor.getVelocity());
        telemetry.addData("Launcher Target Velocity", robot.launcher.velocityTarget);
        telemetry.addData("Launcher at Speed", robot.launcher.motorSpeedCheck(robot.launcher.velocityTarget));
        telemetry.addData("Launcher on Cooldown", robot.launcher.onCooldown);
        telemetry.addData("Blender State", robot.sorterHardware.currentPositionState);
        telemetry.addData("Door Cooldown", robot.sorterHardware.cooldownTimer.seconds());
        telemetry.addData("Launcher Cooldown", robot.launcher.cooldownTimer);

        //robot.tellMotorOutput();
    }
    private float getLargestAbsVal(float[] values){
        // This function does some math!
        float max = 0;
        for (float val : values) {
            if (Math.abs(val) > max) {
                max = Math.abs(val);
            }
        }
        return max;
    }

    private boolean isEven(int x) {
        return x % 2 == 0;
    }
}


