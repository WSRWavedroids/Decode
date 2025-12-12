package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Autonomous.BetaBlueFrontAuto.Step.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.Robot;

import java.util.Objects;

/**
 * This is an iterative autonomous program. It runs in a state machine, which allows us to run the
 * updateAllDaThings() function and properly run the blender without any... questionable code. And
 * it's marginally more efficient. Also, this contains the proper code to run the red alliance auto
 * too, allowing us to keep both autos up to date in a single file. BetaRedFrontAuto is a shell that
 * basically just hijacks this file to work, which is neat.
 */
@Autonomous(group = "Basic", name = "BETA Blue Front")
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

    enum Step {
        START,
        CHECK_MOVE_1, CHECK_MOVE_2, CHECK_TAG, TAG_TELEMETRY, SET_APRILTAG_PIPELINE,
        FIRST_SPIN, LAUNCHER_ON, TURN_BACK_TOWARDS_GOAL, FINE_TUNE_TARGETING, DRIVE_CLOSER_TO_GOAL,
        FIRE_FIRST_PATTERN, RESET_BLENDER,
        UNPARK_1, UNPARK_2, UNPARK_3,
        YAY
    }
    private Step currentStep = START;

    public static final String ALLIANCE_KEY = "Alliance"; //For blackboard
    public static final String PATTERN_KEY = "Pattern";

    /**
     * Code to run ONCE when the driver hits INIT
     */
    public void init() {

        // Call the initialization protocol from the Robot class.
        // Go find pizza
        robot = new Robot(hardwareMap, telemetry, this);
        auto = new AutonomousPlusPLUS(robot);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        robot.randomizationScanner.InitLimeLight(0);
        blackboard.put(ALLIANCE_KEY, "BLUE");
    }

    /**
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    public void init_loop() {
        telemetry.addData("HYPE", "ARE! YOU! READY?!?!?!?!");

        robot.pattern = robot.randomizationScanner.GetRandomization();
        telemetry.addData(robot.pattern, " Works!");
        telemetry.update();
    }

    /**
     * Code to run ONCE when the driver hits PLAY
     */
    public void start() {
        runtime.reset();
        telemetry.addData("HYPE", "Let's do this!!!");
        robot.sorterHardware.resetSorterEncoder();//REMOVE ONCE AUTO -> TELE IS FIGURED OUT
        robot.sorterHardware.legalToSpin = true;
    }

    /**
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    public void loop() {
        switch (currentStep) {
            case START:
                nextStep(CHECK_MOVE_1);
                break;
            case CHECK_MOVE_1:
                auto.moveRobotForward(1000);
                nextStep(CHECK_MOVE_2);
                break;
            case CHECK_MOVE_2:
                if (auto.checkMovement()) {
                    if (Objects.equals(blackboard.get(ALLIANCE_KEY), "BLUE")) {
                        auto.turnRobotLeft(600);
                    } else {
                        auto.turnRobotRight(600);
                    }
                    nextStep(CHECK_TAG);
                }
                break;
            case CHECK_TAG:
                if (auto.checkMovement()) {
                    robot.pattern = robot.randomizationScanner.GetRandomization();
                    robot.sorterHardware.legalToSpin = true;
                    nextStep(SET_APRILTAG_PIPELINE);
                }
                break;
            case TAG_TELEMETRY: // Skipped
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

                nextStep(SET_APRILTAG_PIPELINE);
                break;
            case SET_APRILTAG_PIPELINE:
                if (blackboard.get(ALLIANCE_KEY) == "BLUE") {
                    robot.targetScanner.InitLimeLightTargeting(2, robot);
                    robot.scanningForTargetTag = true;
                } else if(blackboard.get(ALLIANCE_KEY) == "RED") {
                    robot.targetScanner.InitLimeLightTargeting(1, robot);
                    robot.scanningForTargetTag = true;
                } else {
                    robot.targetScanner.InitLimeLightTargeting(1, robot);
                    robot.scanningForTargetTag = true;
                }

                nextStep(FIRST_SPIN);
                break;
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
                break;
            case LAUNCHER_ON:
                if (robot.sorterHardware.positionedCheck()) {
                    robot.launcher.setLauncherSpeed(1);
                    robot.targetTag = robot.targetScanner.tagInfo();
                    nextStep(TURN_BACK_TOWARDS_GOAL);
                }
                break;
            case TURN_BACK_TOWARDS_GOAL:
                if (Objects.equals(blackboard.get(ALLIANCE_KEY), "BLUE")) {
                    auto.turnRobotRight(600);
                } else {
                    auto.turnRobotLeft(600);
                }

                nextStep(FINE_TUNE_TARGETING);
                break;
            case FINE_TUNE_TARGETING:
                if (auto.checkMovement()) {
                    if (robot.targetTag.currentlyDetected) //Angle detect if possible / needed
                    {
                        auto.turnRobotRight((int) ((robot.targetTag.angleX +robot.limelightSideOffsetAngle) * ( (double) 1660 / 360)));
                    }
                    nextStep(DRIVE_CLOSER_TO_GOAL);
                }
                break;
            case DRIVE_CLOSER_TO_GOAL:
                if (auto.checkMovement()) {
                    auto.moveRobotForward(200);
                }
            case FIRE_FIRST_PATTERN:
                if (auto.checkMovement()) {
                    if (robot.pattern.equals("PPG")) {
                        auto.fireInSequence(robot.sorterLogic.slotB, robot.sorterLogic.slotC, robot.sorterLogic.slotA);
                    } else if (robot.pattern.equals("PGP")) {
                        auto.fireInSequence(robot.sorterLogic.slotB, robot.sorterLogic.slotA, robot.sorterLogic.slotC);
                    } else {
                        auto.fireInSequence(robot.sorterLogic.slotA, robot.sorterLogic.slotB, robot.sorterLogic.slotC);
                    }

                    if (auto.fireInSequenceComplete()) {
                        nextStep(RESET_BLENDER);
                    }
                }
                break;
            case RESET_BLENDER:
                robot.sorterHardware.prepareNewMovement(robot.sorterHardware.motor.getCurrentPosition(),
                        robot.sorterLogic.slotA.getLoadPosition());
                nextStep(UNPARK_1);
                break;
            case UNPARK_1:
                if (Objects.equals(blackboard.get(ALLIANCE_KEY), "BLUE")) {
                    auto.turnRobotRight(-1200);
                } else {
                    auto.turnRobotLeft(-1200);
                }

                nextStep(UNPARK_2);
                break;
            case UNPARK_2:
                if (auto.checkMovement()) {
                    robot.launcher.setLauncherSpeed(0);
                    if (Objects.equals(blackboard.get(ALLIANCE_KEY), "BLUE")) {
                        auto.moveRobotLeft(800);
                    } else {
                        auto.moveRobotRight(800);
                    }

                    nextStep(UNPARK_3);
                }
                break;
            case UNPARK_3:
                if (auto.checkMovement()) {
                    auto.moveRobotForward(200);
                    nextStep(YAY);
                }
                break;
            case YAY:
                super.requestOpModeStop();
                robot.telemetry.addLine("YAY");
                break;
        }

        robot.updateAllDaThings();
        doTelemetryStuff();

    }

    /**
     * Code to run ONCE after the driver hits STOP
     */
    public void stop() {
        telemetry.addData("Status", "Robot Stopped");
        doTelemetryStuff();
    }


    /*
     * The holding cell for all of the random functions we call above.
     */
    private void nextStep(Step nextStep) {
        currentStep = nextStep;
    }

    private void nextPatternSpecificStep(Step nextGeneralStep) {

    }

    private void doTelemetryStuff() {
        // This little section updates the driver hub on the runtime and the motor powers.
        // It's mostly used for troubleshooting.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Step", currentStep);
        telemetry.addData("fireInSequenceStallingState", auto.fireInSequenceStallingState);

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


