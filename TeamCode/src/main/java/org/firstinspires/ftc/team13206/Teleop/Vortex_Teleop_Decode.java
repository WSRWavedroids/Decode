package org.firstinspires.ftc.team13206.Teleop;

import static org.firstinspires.ftc.team13206.Core.ArtifactLocator.SlotState.*;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.*;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.*;
import static org.firstinspires.ftc.team13206.Core.Robot.allianceSides.*;
import static org.firstinspires.ftc.team13206.Core.Robot.patternColors.*;
import static org.firstinspires.ftc.team13206.Core.SorterHardware.FeederState.*;
import static org.firstinspires.ftc.team13206.Core.Robot.DriveMode.*;
import static org.firstinspires.ftc.team13206.Core.SorterHardware.PositionState.*;
import static org.firstinspires.ftc.team13206.Core.TurretLogic.controlMode.*;

import com.bylazar.panels.Panels;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team13206.Core.ArtifactLocator;
import org.firstinspires.ftc.team13206.Core.FramerateCalculator;
import org.firstinspires.ftc.team13206.Core.Robot;
import org.firstinspires.ftc.team13206.Core.TurretLogic;
import org.firstinspires.ftc.team13206.Core.fireQueueWithStates;


import java.util.Objects;

/**
 * This file is our iterative (Non-Linear) "OpMode" for TeleOp.
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is selected on the Robot Controller and executed.
 * This OpMode controls the functions of the robot during the driver-controlled period.
 * <p>
 * If the "@Disabled" line is not commented out, the program will not show up on the driver hub.
 * If you ever have problems with the program not showing up on the driver hub, it's probably because of that.
 * <p>
 */
@TeleOp(name = "Fabio", group = "CompBot")
public class Vortex_Teleop_Decode extends OpMode {

    // This section tells the program all of the different pieces of hardware that are on our robot that we will use in the program.
    private ElapsedTime runtime = new ElapsedTime();
    private final FramerateCalculator fps = new FramerateCalculator(runtime);
    private double speed = 0.75;
    private boolean spinTargetAcquired = false;

    private boolean autoDriver2Mode = false;

    private boolean automatedDrive;
    boolean cadenHoldingReady = false;
    boolean cadenHoldingFire = false;

    int SpinTargetFrontLeft;
    int SpinTargetFrontRight;
    int SpinTargetBackLeft;
    int SpinTargetBackRight;

    Follower teleFollower;

    //Trackpad vars for automove
    public double trackpadXMax = 1920;
    public double trackpadXMin = 0;
    public double trackpadYMax = 1020;
    public double trackpadYMin = 0;

    public double trackpadCurrentX;
    public double trackpadCurrentY;

    public double stickerOffsetX = -0.01;
    public double stickerOffsetY = 0.01;

    public Pose trackTarget;


    int slot = 0; // temp for testing lol

    //private double storedSpeed;
    public Robot robot = null;
    public IMU imu;

    private boolean blackboardPositioningSucessful = true;


    public static final String ALLIANCE_KEY = "Alliance"; //For blackboard
    public static final String PATTERN_KEY = "Pattern";

    public boolean canManuallyControlVerticalSlides = true;

    ElapsedTime outtakeTimer = new ElapsedTime();
    boolean killSwitchActivated = false;

    //static TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    private Pose startingPose;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    public void init() {

        // Call the initialization protocol from the Robot class.
        robot = new Robot(hardwareMap, telemetry, this);
        teleFollower = robot.turret.follower;
        robot.targetScanner.InitLimeLightTargeting(1, robot);
        robot.controlMode = PEDRO;
        imu = hardwareMap.get(IMU.class, "imu");
//        robot.turret.rawSwivelController.tolerance = 0;


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        outtakeTimer.reset();

        //if using field centric youl need this lolzeez
        if (Objects.equals(blackboard.get(ALLIANCE_KEY), "RED")) {
            robot.targetScanner.InitLimeLightTargeting(1, robot);
            robot.alliance = RED;
            robot.scanningForTargetTag = true;
        } else if (Objects.equals(blackboard.get(ALLIANCE_KEY), "BLUE")) {
            robot.targetScanner.InitLimeLightTargeting(2, robot);
            robot.alliance = BLUE;
            robot.scanningForTargetTag = true;
        } else {
            robot.targetScanner.InitLimeLightTargeting(2, robot);
            robot.alliance = BLUE;
            robot.scanningForTargetTag = true;
        }

        if (blackboard.get(PATTERN_KEY) == "GPP") {
            robot.pattern = GPP;
        } else if (blackboard.get(PATTERN_KEY) == "PGP") {
            robot.pattern = Robot.patternColors.PGP;
        } else if ((blackboard.get(PATTERN_KEY) == "PPG")) {
            robot.pattern = PPG;
        } else {
            robot.pattern = PPG;
        }

        robot.panels = Panels.INSTANCE;
        robot.readyHardware(true);//Might not be needed

        grabStartPose();

        robot.robotPosition.x = startingPose.getX();
        robot.robotPosition.y = startingPose.getY();
        robot.robotHeading = startingPose.getHeading();
        teleFollower.setPose(startingPose);
        teleFollower.setHeading(startingPose.getHeading());

        robot.turret.blackboardSafe = false;
    }

    private void grabStartPose() {
        Object pedroXFromBB = blackboard.getOrDefault("PedroX", 72.0);
        Object pedroYFromBB = blackboard.getOrDefault("PedroY", 72.0);
        Object pedroHeadingFromBB =  blackboard.getOrDefault("PedroHeading", Math.PI / 2);

        double goodX = 72;
        double goodY = 72;
        double goodHeading = Math.PI / 2;

        if (pedroXFromBB instanceof Number) {
            goodX = (double) pedroXFromBB;
        } else {
            blackboardPositioningSucessful = false;
        }
        if (pedroYFromBB instanceof Number) {
            goodY = (double) pedroYFromBB;
        } else {
            blackboardPositioningSucessful = false;
        }
        if (pedroHeadingFromBB instanceof Number) {
            goodHeading = (double) pedroHeadingFromBB;
        } else {
            blackboardPositioningSucessful = false;
        }
        startingPose = new Pose(goodX, goodY, goodHeading);
    }

    /**
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    public void init_loop() {
        telemetry.addData("HYPE", "ARE! YOU! READY?!?!?!?!");
        //doTelemetryStuff();
        telemetry.update();
    }

    /**
     * Code to run ONCE when the driver hits PLAY
     */
    public void start() {
        runtime.reset();
        telemetry.addData("HYPE", "Let's do this!!!");
        gamepad1.setLedColor(0, 0, 255, 10);
        gamepad2.setLedColor(0, 0, 255, 10);
        robot.sorterHardware.calibrate();
        robot.sorterHardware.legalToSpin = true;

        robot.frontLeftDrive.setZeroPowerBehavior(BRAKE);
        robot.frontRightDrive.setZeroPowerBehavior(BRAKE);
        robot.backLeftDrive.setZeroPowerBehavior(BRAKE);
        robot.backRightDrive.setZeroPowerBehavior(BRAKE);
    }

    /**
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    public void loop() {

        robot.updateAllDaThings();

        //So Begins the input chain. At least try a bit to organise by driver

        if (gamepad2.left_stick_y >= 0.9) {
            autoDriver2Mode = true;
            TurretLogic.activeMode = FULL;
        } else if (gamepad2.left_stick_y <= -0.5) {
            autoDriver2Mode = false;
        }

        driveSpeed();

        controlMode();

        controllerRumble();

        if(robot.controlMode == PEDRO)
        {
            teleFollower.update();
            pedroAutomation();
        }
        else
        {
            singleJoystickDrive();
        }

        if (autoDriver2Mode) {
            /*
            Welcome to blindfold mode! This behavior emulates Caden when he's driving blindfolded.
            It can't sort, but we weren't doing that 75% of the match anyway.
             */

            // First, let's make sure we aren't actively firing
            if (robot.queue.noBallsQueued) {
                robot.sorterHardware.runAdvancedIntake(); // Run the intake if at all possible

                // If we're in a firing zone, and we aren't firing, fix that
                if (robot.sorterLogic.inventory.getTotalCount() == 3 & robot.zoneDetector.isInFireZone()) {
                    robot.queue.fillSimple();
                }
            }
        } else {
            fireAll();

            intake();

            launcherToggle();

            fireCurrentFireSlot();

            firePatternWithOffset();

            incrementThroughPositions();

            toggleTurretFullMode();
        }


        switchAlliance();

        //turretAssist();

        resetPedroPosition();

        runTrackpadFunctions();

        //robot.panelsTelemetry.addData("Motor Position", robot.launcher.motor.getCurrentPosition());
        robot.panelsTelemetry.update();

        //fps.update();

        if (gamepad2.right_stick_button) {
            doTelemetryStuff();
        }


        if (gamepad2.touchpad) {
            killSwitchActivated = true;
            requestOpModeStop();
        }
    }

    /**
     * Code to run ONCE after the driver hits STOP
     */
    public void stop() {
        telemetry.addData("Status", "Robot Stopped");
        blackboard.put("PedroX", robot.robotPosition.x);
        blackboard.put("PedroY", robot.robotPosition.y);
        blackboard.put("PedroHeading", Math.toRadians(robot.robotHeading));
        if (killSwitchActivated){
            telemetry.addLine("Killswitch Hit!");
        }
    }


    /*
     * The holding cell for all of the random functions we call above.
     */

    private void colorMovement() {
        if (gamepad2.square && !gamepad2.left_bumper) {
            robot.sorterHardware.prepareNewMovement(
                    robot.sorterLogic.findBestPositionedType(PURPLE, FIRE).getFirePosition());
        } else if (gamepad2.triangle && !gamepad2.left_bumper) {
            robot.sorterHardware.prepareNewMovement(
                    robot.sorterLogic.findBestPositionedType(GREEN, FIRE).getFirePosition());
        }
    }

    private void fireQueue() {
        /// Clears list each time the button is deliberately pressed, so ready for queueing
        /// Without this we have no way to empty it without firing
        if (gamepad2.leftBumperWasPressed()) {
            robot.queue.clearList();
        }

        /// Adds color to queue
        if (gamepad2.squareWasPressed() && gamepad2.left_bumper) {
            robot.queue.addToNextSpotColor(PURPLE);
            gamepad2.setLedColor(152, 7, 224, 100);
        } else if (gamepad2.triangleWasPressed() && gamepad2.left_bumper) {
            robot.queue.addToNextSpotColor(GREEN);
            gamepad2.setLedColor(0, 255, 0, 100);
        }
    }
    private void fireAll() {
        if (gamepad2.rightBumperWasPressed()) {
            if (robot.queue.checkForExistingQueue()) {
                robot.queue.wantToFireQueue = fireQueueWithStates.firingQueue.SMART;
            } else if (robot.sorterLogic.inventory.canMakePattern()) {
                robot.queue.clearList();
                robot.queue.addPattern(robot.pattern);
                robot.queue.wantToFireQueue = fireQueueWithStates.firingQueue.SMART;
            } else {
                robot.queue.clearList();
                robot.queue.fillSimple(); // replace with the if when cam ready
                robot.queue.wantToFireQueue = fireQueueWithStates.firingQueue.DUMB;
            }
        }
    }

    private void firePatternWithOffset() {
        if (gamepad2.leftBumperWasPressed()) {
            robot.queue.addOffsetPattern(robot.pattern, 0);
        } else if (gamepad2.squareWasPressed()) {
            robot.queue.addOffsetPattern(robot.pattern, 1);
        } else if (gamepad2.triangleWasPressed()) {
            robot.queue.addOffsetPattern(robot.pattern, 2);
        }
    }

    private void intake() {
        if (gamepad2.cross) {
            robot.sorterHardware.runAdvancedIntake();
        } else if (gamepad2.circle) // dave spits out artifact
        {
            robot.runBasicIntake(-1);
            robot.sorterHardware.setFeeders(OUTTAKE);
        } else //don't run intake if we not pulling trigger
        {
            robot.cancelAutoIntake();
            robot.runBasicIntake(0.01); //Always keep a slight power flow to servos to prevent input delay from module
        }
    }

    private void controllerRumble() {
        if (robot.sorterLogic.inventory.getTotalCount() >= 3) {
            gamepad1.rumble(1, 1, 100);
        }

        if (robot.launcher.isInFireSequence()) {
            gamepad2.rumble(0.5, 0, 50);
        }
    }

    private void launcherToggle() {
        if (gamepad2.left_trigger > 0.50) {

            if (!cadenHoldingReady) {
                cadenHoldingReady = true;
            }

            if (!robot.launcher.launcherOn()) {
                robot.launcher.setPerfectLauncherVelocity();
            } else {
                // Will stop motor
                robot.queue.finishQueue();
            }

        } else {
            cadenHoldingReady = false;
        }
    }

    private void fireCurrentFireSlot() {
        if (gamepad2.right_trigger > 0.50 && !robot.launcher.isInFireSequence() /*&& robot.queue.wantToFireQueue == fireQueueWithStates.firingQueue.NONE*/) {
            if (!cadenHoldingFire) {
                cadenHoldingFire = true;
                robot.launcher.fireWithinTimeIfSafe(0.5, false, false, 0.5);
            }
        } else {
            cadenHoldingFire = false;
        }
    }


    private void toggleTurretFullMode() {
        if (gamepad2.leftStickButtonWasPressed()) {
            switch (TurretLogic.activeMode) {
                case LOCKED:
                    TurretLogic.activeMode = FULL;
                    break;
                case PARTIAL:
                case OVERIDE:
                case FULL:
                    TurretLogic.activeMode = LOCKED;
            }
        }
    }

    public void setIndividualPowers(float[] motorPowers) {
        // This function creates an array so that the function below works.
        // Don't mess with this function unless you know what you're doing.

        if (motorPowers.length != 4) {
            return;
        }
        robot.frontLeftDrive.setMode(RUN_WITHOUT_ENCODER);
        robot.frontRightDrive.setMode(RUN_WITHOUT_ENCODER);
        robot.backLeftDrive.setMode(RUN_WITHOUT_ENCODER);
        robot.backRightDrive.setMode(RUN_WITHOUT_ENCODER);

        robot.frontLeftDrive.setPower(motorPowers[0]);
        robot.frontRightDrive.setPower(motorPowers[1]);
        robot.backLeftDrive.setPower(motorPowers[2]);
        robot.backRightDrive.setPower(motorPowers[3]);
    }

    private void autoWheel(boolean detected, double anglex) {

        if (gamepad1.left_bumper)//Does 180
        {
            SpinTargetFrontLeft = robot.frontLeftDrive.getCurrentPosition() + 830 * 2;
            SpinTargetFrontRight = robot.frontRightDrive.getCurrentPosition() - 830 * 2;
            SpinTargetBackLeft = robot.backLeftDrive.getCurrentPosition() + 830 * 2;
            SpinTargetBackRight = robot.backRightDrive.getCurrentPosition() - 830 * 2;
            spinTargetAcquired = true;
            speed = 1;
        }

        if ((gamepad1.right_bumper && !detected) || gamepad1.square) {
            robot.frontLeftDrive.setTargetPosition(SpinTargetFrontLeft);
            robot.frontRightDrive.setTargetPosition(SpinTargetFrontRight);
            robot.backLeftDrive.setTargetPosition(SpinTargetBackLeft);
            robot.backRightDrive.setTargetPosition(SpinTargetBackRight);
        } else if (detected && gamepad1.right_bumper) //Turn to face target tag
        {
            double constant = (double) -1660 / 360; //We will know this later
            speed = 1;
            int turnTicks = (int) ((robot.targetTag.angleX + robot.limelightSideOffsetAngle) * constant);

            robot.frontLeftDrive.setTargetPosition(robot.frontLeftDrive.getCurrentPosition() + turnTicks);
            robot.frontRightDrive.setTargetPosition(robot.frontRightDrive.getCurrentPosition() - turnTicks);
            robot.backLeftDrive.setTargetPosition(robot.backLeftDrive.getCurrentPosition() + turnTicks);
            robot.backRightDrive.setTargetPosition(robot.backRightDrive.getCurrentPosition() - turnTicks);
        }

        robot.setRunMode(RUN_TO_POSITION);
        robot.powerSet(speed);
    }

    private void singleJoystickDrive() {
        // We don't really know how this function works, but it makes the wheels drive, so we don't question it.
        // Don't mess with this function unless you REALLY know what you're doing.
        float leftY = -this.gamepad1.left_stick_y;
        float rightX = -this.gamepad1.right_stick_x;
        float leftX = -this.gamepad1.left_stick_x;


        float[] motorPowers = new float[4];

        if (robot.controlMode != LEGACY_FIELD_CENTRIC) {
            motorPowers[0] = (leftY - leftX - rightX); // frontLeftDrive
            motorPowers[1] = (leftY + leftX + rightX); // frontRightDrive
            motorPowers[2] = (leftY + leftX - rightX); // backLeftDrive
            motorPowers[3] = (leftY - leftX + rightX);
        }
        // backRightDrive


        else {
            double rotX = leftX * Math.cos(-robot.robotHeading) - leftY * Math.sin(-robot.robotHeading);
            double rotY = leftX * Math.sin(-robot.robotHeading) + leftY * Math.cos(-robot.robotHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rightX), 1);
            motorPowers[0] = (float) -((rotY + rotX + rightX) / denominator);
            motorPowers[1] = (float) -((rotY - rotX + rightX) / denominator);
            motorPowers[2] = (float) -((rotY - rotX - rightX) / denominator);
            motorPowers[3] = (float) -((rotY + rotX - rightX) / denominator);
        }

        float max = getLargestAbsVal(motorPowers);
        if (max < 1) {
            max = 1;
        }

        for (int i = 0; i < motorPowers.length; i++) {
            motorPowers[i] *= (1 / max);

            float abs = Math.abs(motorPowers[i]);
            if (abs < 0.05) {
                motorPowers[i] = 0.0f;
            }
            if (abs > 1.0) {
                motorPowers[i] /= abs;
            }
        }

        setIndividualPowers(motorPowers);
    }

    private void controlMode() {
        if (gamepad1.back && !gamepad1.start) {
            if (robot.controlMode == ROBOT_CENTRIC) {
                robot.controlMode = PEDRO;
            } else if (robot.controlMode == PEDRO) {
                robot.controlMode = ROBOT_CENTRIC;
            }
        }

        if (gamepad1.options && robot.controlMode == PEDRO) {
            imu.resetYaw();
        }
    }

    private void manualTuneLauncher() {
        autoDriver2Mode = false;
        if (gamepad2.dpadUpWasPressed()) {
            robot.launcher.velocityTarget += 20;
        } else if (gamepad2.dpadDownWasPressed()) {
            robot.launcher.velocityTarget -= 20;
        }
    }

    private void driveSpeed() {
        if (gamepad1.dpad_up || gamepad1.right_trigger >= 0.25) {
            speed = 1;
        } else if (gamepad1.dpad_down) {
            speed = 0.25;
        } else if (gamepad1.dpad_left || gamepad1.left_trigger >= 0.25) {
            speed = 0.5;
        } else if (gamepad1.dpad_right) {
            speed = 0.75;
        }
    }

    private void incrementThroughPositions() {
        // Fire positions
        if (gamepad2.dpadLeftWasPressed()) {
            goNextPosition(-1);
        } else if (gamepad2.dpadRightWasPressed()) {
            goNextPosition(1);
        }
    }

    private void goNextPosition(int go) {
        robot.sorterHardware.prepareNewMovement(
                robot.sorterLogic.offsetPositions.get(
                        makeSureNewOffsetIsOK(
                                robot.sorterLogic.findClosestOffset(
                                        robot.sorterHardware.motor.getCurrentPosition()) + go
                        )
                )
        );
    }

    private int makeSureNewOffsetIsOK(int offset) {
        while (offset < 0) {
            offset += 3;
        }
        while (offset > 2) {
            offset -= 3;
        }
        return offset;
    }

    private void switchAlliance() {
        if (gamepad2.start && gamepad2.shareWasPressed()) {
            if (robot.alliance == BLUE) {
                robot.alliance = RED;
                robot.targetScanner.InitLimeLightTargeting(1, robot);
            } else {
                robot.alliance = BLUE;
                robot.targetScanner.InitLimeLightTargeting(2, robot);
            }
        }
    }

    private void resetPedroPosition() {
        if (gamepad1.start) {
            boolean pressed = gamepad1.shareWasPressed();
            if (pressed && robot.alliance == BLUE) {
                teleFollower.setPose(new Pose(133.28, 10.75));
                teleFollower.setHeading(Math.PI / 2);

                robot.turret.updateTurretPositionXY();
            }
            if (pressed && robot.alliance == RED) {
                teleFollower.setPose(new Pose(9.59, 9.67));
                teleFollower.setHeading(Math.PI / 2);
                robot.turret.updateTurretPositionXY();
            }
        }
    }

    private void doTelemetryStuff() {
        // This little section updates the driver hub on the runtime and the motor powers.
        // It's mostly used for troubleshooting.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Alliance", robot.alliance);
        //telemetry.addData("Framerate (last 15 seconds)", fps.getFramerate(30) + " fps");

        telemetry.addLine();
        telemetry.addLine("Artifact Storage:");
        telemetry.addData("Total Inventory", robot.sorterLogic.inventory.getTotalCount());
        telemetry.addLine("Purple: " + robot.sorterLogic.inventory.getPurpleCount() +
                " Green: " + robot.sorterLogic.inventory.getGreenCount());
        telemetry.addData("Slot A", robot.sorterLogic.slotA.getOccupied());
        telemetry.addData("Slot B", robot.sorterLogic.slotB.getOccupied());
        telemetry.addData("Slot C", robot.sorterLogic.slotC.getOccupied());

        telemetry.addLine();
        telemetry.addLine("Blender:");
        telemetry.addData("Current Position", robot.sorterHardware.motor.getCurrentPosition());
        telemetry.addData("Target Position", robot.sorterHardware.reference);
        telemetry.addData("Blender in position", robot.sorterHardware.positionedCheck());

        telemetry.addLine();
        telemetry.addLine("Launcher:");
        telemetry.addData("Launcher Velocity", robot.launcherMotorOne.getVelocity());
        telemetry.addData("Launcher Target Velocity", robot.launcher.velocityTarget);
        telemetry.addData("Launcher at Speed", robot.launcher.motorSpeedCheck(robot.launcher.velocityTarget));
        telemetry.addData("LL Distance", robot.targetTag.distanceZ);

        telemetry.addData("Currently Firing", robot.launcher.activeFiringSlot.getName());
        telemetry.addData("Fire Color Queue", robot.queue.ballQueue);
        telemetry.addData("Fire Slot Queue", ArtifactLocator.getNamesOfSlots(robot.queue.slotQueue));

        telemetry.addLine();
        telemetry.addLine("Blackboard:");
        telemetry.addData("Last saved pattern", blackboard.get(PATTERN_KEY));
        telemetry.addData("Last saved Alliance", blackboard.get(ALLIANCE_KEY));
        telemetry.addData("Position grab successful", blackboardPositioningSucessful);

        telemetry.addLine();
        telemetry.addData("Blindfold Mode", autoDriver2Mode);

        //robot.tellMotorOutput();
    }

    private float getLargestAbsVal(float[] values) {
        // This function does some math!
        float max = 0;
        for (float val : values) {
            if (Math.abs(val) > max) {
                max = Math.abs(val);
            }
        }
        return max;
    }

    private void turretAssist() {
        if (gamepad2.right_stick_button) {
            robot.turret.input = gamepad2.right_stick_x;
        } else {
            robot.turret.input = 0;
        }
    }

    void runTrackpadFunctions() {
        if (gamepad1.touchpad_finger_1) {
            trackpadCurrentX = gamepad1.touchpad_finger_1_x;
            trackpadCurrentY = gamepad1.touchpad_finger_1_y; // Corrected for inversion
            trackTarget = translateTrackpad(trackpadCurrentX, trackpadCurrentY, ""); // Sets tracktarget to coords

            telemetry.addData("Finger 1 x detected val: ", gamepad1.touchpad_finger_1_x);
            telemetry.addData("Finger 1 y detected val: ", gamepad1.touchpad_finger_1_y);

            telemetry.addData("Finger 1 x adjusted: ", trackpadCurrentX);
            telemetry.addData("Finger 1 y adjusted: ", trackpadCurrentY);

            telemetry.addData("Pedro Target Position: ", trackTarget);
        } else if (trackTarget == null) {
            trackTarget = new Pose(72, 72, 0);
        }
    }

    private Pose translateTrackpad(double inX, double inY, String headingCheck) {


        //fix y axis inversion (top is 0 instead of bottom)
        //inY = Math.abs(inY - trackpadYMax);

        //if the heading check is tag rotate to point at target during path
        if (headingCheck == "tag") {
            return new Pose(((inX) * 72) + 72, ((inY) * 72) + 72);
        } else //or just keep current heading for same movement
        {
            return new Pose(((inX) * 72) + 72, ((inY) * 72) + 72);
        }
    }

    private PathChain makeDynamicPath(Pose targetPose, double targetHeadingDegrees) {

        return teleFollower.pathBuilder()
                .addPath(new BezierLine(teleFollower.getPose(), targetPose))
                .setLinearHeadingInterpolation(teleFollower.getHeading(), Math.toRadians(targetHeadingDegrees))
                .build();
        // Build the PathChain after adding all paths
    }

    private PathChain makeDynamicChain(Pose initalPose, Pose secondaryPose, double targetHeadingDegrees) {
            return teleFollower.pathBuilder()
                    .addPath(new BezierLine(teleFollower.getPose(), initalPose))
                    .setLinearHeadingInterpolation(teleFollower.getHeading(), Math.toRadians(targetHeadingDegrees))
                    .addPath(new BezierLine(initalPose, secondaryPose))
                    .setLinearHeadingInterpolation(targetHeadingDegrees, targetHeadingDegrees)
                    .build();
            // Build the PathChain after adding all paths
        }

    private void pedroAutomation() {
        //Stop automated following when the driver needs to
        if (automatedDrive && (gamepad1.circle || Math.abs(gamepad1.left_stick_x) >= 0.25
                || Math.abs(gamepad1.left_stick_y) >= 0.25 || Math.abs(gamepad1.right_stick_x) >= 0.25)) {
            teleFollower.startTeleopDrive();
            automatedDrive = false;
        }

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors
            singleJoystickDrive();
        }

        teleFollower.setMaxPower(1);
        //Automated PathFollowing
        speed = 1;

        if (gamepad1.touchpadWasPressed()) {
            teleFollower.followPath(makeDynamicPath(trackTarget, teleFollower.getHeading()));
            automatedDrive = true;
        }
        else if (gamepad1.squareWasPressed()) { //Hold Position and heading
            teleFollower.holdPoint(new BezierPoint(teleFollower.getPose()), teleFollower.getHeading());

            speed = 1;
            automatedDrive = true;
        }
        else if (gamepad1.left_stick_button) { //Autopark
            Pose park;
            if(robot.alliance == BLUE)
            {

                park = new Pose (104, 35, 180);
            }
            else
            {
                park = new Pose (38.6, 33.4, 180);
            }
            teleFollower.followPath(makeDynamicPath(park, 180));


            speed = 1;
            automatedDrive = true;
        }
        else if (gamepad1.dpad_up  && robot.alliance != null) { //auto gate
            Pose pregate;
            Pose pressing;

            double gateHeading;
            double distance;
            double backOfRobotHeading;

            if(robot.alliance == BLUE)
            {
                backOfRobotHeading = 0;
            }
            else
            {
                backOfRobotHeading = 180;
            }

            gateHeading = backOfRobotHeading;
            distance = Math.abs(robot.robotHeading - backOfRobotHeading);
            if(Math.abs(robot.robotHeading - 270) < distance)
            {
                gateHeading = 270;
                distance = Math.abs(robot.robotHeading - 270);
            }
            if(Math.abs(robot.robotHeading - 90) < distance)
            {
                gateHeading = 90;
            }

            if(robot.alliance == BLUE)
            {
                pregate = new Pose (22, 70, gateHeading);
                pressing = new Pose (16.5,70, gateHeading);
            }
            else
            {
                pregate = new Pose (122, 70, gateHeading);
                pressing = new Pose (127.5,70, gateHeading);
            }

            teleFollower.followPath(makeDynamicChain(pregate, pressing, gateHeading));
            speed = 1;
            automatedDrive = true;
        }


    }

    private boolean isEven(int x) {
        return x % 2 == 0;
    }

    private double roundToThousandths(double input) {
        return Math.round(input * 1000.0) / 1000.0;
    }
}


