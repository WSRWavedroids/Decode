package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.teamcode.Core.ArtifactLocator.SlotState.GREEN;
import static org.firstinspires.ftc.teamcode.Core.ArtifactLocator.SlotState.PURPLE;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.*;
import static org.firstinspires.ftc.teamcode.Core.SorterHardware.positionState.*;

import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
@TeleOp(name = "DAVE", group = "CompBot")
public class Vortex_Teleop_Decode extends OpMode {

    // This section tells the program all of the different pieces of hardware that are on our robot that we will use in the program.
    private ElapsedTime runtime = new ElapsedTime();
    private double speed = 0.75;
    private boolean spinTargetAcquired = false;

    private boolean cadenRecording = false;
    private boolean contTwoBumpersPressed = false;
    boolean cadenON = false;
    boolean cadenHoldingReady = false;
    boolean cadenHoldingFire = false;

    int SpinTargetFrontLeft;
    int SpinTargetFrontRight;
    int SpinTargetBackLeft;
    int SpinTargetBackRight;

    int slot = 0; // temp for testing lol

    int targetOffset = 0;

    //private double storedSpeed;
    public Robot robot = null;
    public IMU imu;




    public static final String ALLIANCE_KEY = "Alliance"; //For blackboard
    public static final String PATTERN_KEY = "Pattern";

    public boolean canManuallyControlVerticalSlides = true;

    ElapsedTime outtakeTimer = new ElapsedTime();

    static TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    public void init() {

        // Call the initialization protocol from the Robot class.
        robot = new Robot(hardwareMap, telemetry, this);

        robot.targetScanner.InitLimeLightTargeting(1, robot);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        outtakeTimer.reset();

        if (robot.controlMode == "Field Centric") {

            imu = hardwareMap.get(IMU.class, "imu");
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)); //Forward = left fsr
            // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
            imu.initialize(parameters);
        }
        //if using field centric youl need this lolzeez
        if (Objects.equals(blackboard.get(ALLIANCE_KEY), "BLUE")) {
            robot.targetScanner.InitLimeLightTargeting(1, robot);
            robot.scanningForTargetTag = true;
        } else if (Objects.equals(blackboard.get(ALLIANCE_KEY), "RED")) {
            robot.targetScanner.InitLimeLightTargeting(2, robot);
            robot.scanningForTargetTag = true;
        } else {
            robot.targetScanner.InitLimeLightTargeting(2, robot);
            robot.scanningForTargetTag = true;
        }

        robot.panels = Panels.INSTANCE;
        robot.readyHardware(true);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    public void init_loop() {
        telemetry.addData("HYPE", "ARE! YOU! READY?!?!?!?!");
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

        robot.updateAllDaThings();

        //So Begins the input chain. At least try a bit to organise by driver

        //Driver 1
        controlMode();
        driveSpeed();

        if(robot.targetTag.currentlyDetected) {
            gamepad1.rumble(0.25, 0.25, 100);
            //gamepad1.rumble(100);
        }

        if (gamepad1.left_bumper || gamepad1.right_bumper || gamepad1.triangle) {
            autoWheel(robot.targetTag.currentlyDetected, robot.targetTag.angleX);
        } else {
            singleJoystickDrive();
            spinTargetAcquired = false;
        }


        /// This will work once we have inventory Cam
        /// Preps color of choice

        if(gamepad2.square && !gamepad2.left_bumper)
        {
            robot.sorterHardware.prepareNewMovement(robot.sorterHardware.motor.getCurrentPosition(), robot.sorterLogic.findFirstType(PURPLE).getFirePosition());
        }
        else if(gamepad2.triangle && !gamepad2.left_bumper)
        {
            robot.sorterHardware.prepareNewMovement(robot.sorterHardware.motor.getCurrentPosition(), robot.sorterLogic.findFirstType(GREEN).getFirePosition());
        }
        /*else if(gamepad2.left_trigger > 0.5 && !gamepad2.left_bumper && robot.sorterHardware.currentPositionState == LOAD)//ok this might not be great... don't have the button map with me atm
        {
            robot.sorterHardware.prepareNewMovement(robot.sorterHardware.motor.getCurrentPosition(), robot.sorterLogic.findFirstOccupied().getFirePosition());
        }*/

        /// Clears list each time the button is deliberately pressed, so ready for queueing
        /// Without this we have no way to empty it without firing
        if(gamepad2.leftBumperWasPressed())
        {
           robot.queue.clearList();
        }

        /// Adds color to queue
        if(gamepad2.squareWasPressed() && gamepad2.left_bumper)
        {
            robot.queue.addToNextSpotColor(PURPLE);
        }
        else if(gamepad2.triangleWasPressed() && gamepad2.left_bumper)
        {
            robot.queue.addToNextSpotColor(GREEN);
        }


        if(gamepad2.cross)
        {
            if(robot.sorterHardware.currentPositionState == FIRE)
            {
                //if not in load position, go there and make sure we don't jam in the process
                goNextLoadPosition(1);
                robot.cancelAutoIntake();
            }
            else if(robot.sorterHardware.currentPositionState == SWITCH)
            {
                //dont jam while spinning to load
                robot.cancelAutoIntake();
            }
            else
            {
                //intake if we good
                robot.runAutoIntakeSequence();
            }
        }
        else if(gamepad2.circle) // dave spits out artifact
        {
            robot.runBasicIntake(-1);
            robot.sorterHardware.overidingFeeders = true;
        }
        else //dont run intake if we not pulling trigger
        {
           robot.cancelAutoIntake();
           robot.sorterHardware.overidingFeeders = false;
           robot.runBasicIntake(0.01); //Always keep a slight power flow to servos to prevent input delay from module
        }

        //WSeñorMichael

        if(robot.sorterHardware.fireSafeCheck())
        {
           gamepad2.rumble(0.5, 0, 50);
        }
        if(robot.launcher.inSpeedRange)
        {
            gamepad2.rumble(0, 0.5, 50);
        }


        if(gamepad2.left_trigger > 0.50)
        {

            if (!cadenHoldingReady) {
                cadenHoldingReady = true;
                if (cadenON) {
                    cadenON = false;
                } else {
                    cadenON = true;
                }
            }

            if (cadenON) {
                robot.launcher.setLauncherSpeed(1);
            }
            else
            {
                robot.launcher.setLauncherSpeed(0);
            }

        }
        else
        {
            cadenHoldingReady = false;
        }

        if(gamepad2.right_trigger > 0.50 && !robot.launcher.wantToOpenDoor) {
            if(!cadenHoldingFire)
            {
                cadenHoldingFire = true;
                robot.launcher.readyFire(1, false);
            }
        }
        else
        {
            cadenHoldingFire = false;
        }

        if(gamepad2.rightBumperWasPressed())
        {
            robot.queue.clearList();
            robot.queue.fillSimple(); // replace with the if when cam ready


            /*
            //automatically sets up the pattern in the queue if its possible to do so...
            //... and if caden hasn't already made a list of his own

            if(robot.sorterLogic.inventory.purpleCount == 2 && robot.sorterLogic.inventory.greenCount == 1 && !robot.queue.checkForExistingQueue())
            {
                robot.queue.addPattern(blackboard.get(PATTERN_KEY));
            }*/

        }
        if(gamepad2.right_bumper)
        {
            robot.queue.fireAllDumb(1);//Replace with following line once cam done
            //robot.queue.fireAllSmart(1);


        }


        magicFixEverything();

        incrementThroughPositions();


        telemetry.addData("currentSlot target: ", slot);

        if (gamepad1.touchpad || gamepad2.touchpad) {
            requestOpModeStop();
        }

        //robot.panelsTelemetry.addData("Motor Position", robot.launcher.motor.getCurrentPosition());
        robot.panelsTelemetry.update();

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

        robot.frontLeftDrive.setPower(-motorPowers[0]);
        robot.frontRightDrive.setPower(-motorPowers[1]);
        robot.backLeftDrive.setPower(-motorPowers[2]);
        robot.backRightDrive.setPower(-motorPowers[3]);
    }

    private void autoWheel(boolean detected, double anglex) {


        if (gamepad1.triangle) {//180
            SpinTargetFrontLeft = robot.frontLeftDrive.getCurrentPosition() + 830*2;
            SpinTargetFrontRight = robot.frontRightDrive.getCurrentPosition() - 830*2;
            SpinTargetBackLeft = robot.backLeftDrive.getCurrentPosition() + 830*2;
            SpinTargetBackRight = robot.backRightDrive.getCurrentPosition() - 830*2;
            spinTargetAcquired = true;
            speed = 1;
        }//we so cool if this works

        if(gamepad1.left_bumper)
        {
            SpinTargetFrontLeft = robot.frontLeftDrive.getCurrentPosition() + 830*2;
            SpinTargetFrontRight = robot.frontRightDrive.getCurrentPosition() - 830*2;
            SpinTargetBackLeft = robot.backLeftDrive.getCurrentPosition() + 830*2;
            SpinTargetBackRight = robot.backRightDrive.getCurrentPosition() - 830*2;
            spinTargetAcquired = true;
            speed = 1;
        }


        if (gamepad1.right_bumper && !detected) {
            robot.frontLeftDrive.setTargetPosition(SpinTargetFrontLeft);
            robot.frontRightDrive.setTargetPosition(SpinTargetFrontRight);
            robot.backLeftDrive.setTargetPosition(SpinTargetBackLeft);
            robot.backRightDrive.setTargetPosition(SpinTargetBackRight);
        } else if (detected && gamepad1.right_bumper) //Turn to face target tag
        {
            double constant = -1660 / 360; //We will know this later
            speed = 1;
            int turnTicks = (int) ((robot.targetTag.angleX + robot.limelightSideOffsetAngle) * constant);

            robot.frontLeftDrive.setTargetPosition(robot.frontLeftDrive.getCurrentPosition() + turnTicks);
            robot.frontRightDrive.setTargetPosition(robot.frontRightDrive.getCurrentPosition() - turnTicks);
            robot.backLeftDrive.setTargetPosition(robot.backLeftDrive.getCurrentPosition() + turnTicks);
            robot.backRightDrive.setTargetPosition(robot.backRightDrive.getCurrentPosition() - turnTicks);
        }


        robot.frontLeftDrive.setPower(speed);
        robot.frontRightDrive.setPower(speed);
        robot.backLeftDrive.setPower(speed);
        robot.backRightDrive.setPower(speed);

        robot.frontLeftDrive.setMode(RUN_TO_POSITION);
        robot.frontRightDrive.setMode(RUN_TO_POSITION);
        robot.backLeftDrive.setMode(RUN_TO_POSITION);
        robot.backRightDrive.setMode(RUN_TO_POSITION);
    }

    private void singleJoystickDrive() {
        // We don't really know how this function works, but it makes the wheels drive, so we don't question it.
        // Don't mess with this function unless you REALLY know what you're doing.
        float leftY = this.gamepad1.left_stick_y;
        float rightX = -this.gamepad1.right_stick_x;
        float leftX = -this.gamepad1.left_stick_x;

        double leftStickAngle = Math.atan2(leftY, leftX);
        double leftStickMagnitude = Math.sqrt(leftX * 2.0 + leftY * 2.0);
        //double robotAngle = robot.imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;

        if (leftStickMagnitude > 1){
            leftStickMagnitude = 1;
        }

        float[] motorPowers = new float[4];

        if (robot.controlMode == "Robot Centric") {

            motorPowers[0] = (leftY + leftX + rightX);//might need inverted back
            motorPowers[1] = (leftY - leftX - rightX);
            motorPowers[2] = (leftY - leftX + rightX);
            motorPowers[3] = (leftY + leftX - rightX);

        } else if (robot.controlMode == "Field Centric") {
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);;// sparky.getPosition().h

            // Rotate the movement direction counter to the bot's rotation
            double rotX = leftX * Math.cos(-botHeading) - leftY * Math.sin(-botHeading);
            double rotY = leftX * Math.sin(-botHeading) + leftY * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rightX), 1);
            double frontLeftPower = (rotY + rotX + rightX) / denominator; //all of the right xs got inverted
            double backLeftPower = (rotY - rotX - rightX) / denominator;
            double frontRightPower = (rotY - rotX + rightX) / denominator;
            double backRightPower = (rotY + rotX - rightX) / denominator;



            motorPowers[0] = (float)frontLeftPower;
            motorPowers[1] = (float) backLeftPower;
            motorPowers[2] = (float)frontRightPower;
            motorPowers[3] = (float) backRightPower;
        }


        float max = getLargestAbsVal(motorPowers);
        if (max < 1) {
            max = 1;
        }

        for (int i = 0; i < motorPowers.length; i++) {
            motorPowers[i] *= (speed / max);

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
        if (gamepad1.back) {
            if (robot.controlMode == "Robot Centric"){
                robot.controlMode = "Field Centric";
                telemetry.addData("Control Mode", "Field Centric Controls");
            } else if (robot.controlMode == "Field Centric") {
                robot.controlMode = "Robot Centric";
                telemetry.addData("Control Mode", "Robot Centric Controls");
            }
        }

        if (gamepad1.options && robot.controlMode == "Field Centric") {
            imu.resetYaw();
        }
    }

    private void driveSpeed() {
        if (gamepad1.dpad_up || gamepad1.right_trigger >= 0.5) {
            speed = 1;
        } else if (gamepad1.dpad_down) {
            speed = 0.25;
        } else if (gamepad1.dpad_left || gamepad1.left_trigger >0.5) {
            speed = 0.5;
        } else if (gamepad1.dpad_right) {
            speed = 0.75;
        }

        if (speed == 1) {
            telemetry.addData("Speed", "Fast Boi");
        } else if (speed == 0.5) {
            telemetry.addData("Speed", "Slow Boi");
        } else if (speed == 0.25) {
            telemetry.addData("Speed", "Super Slow Boi");
        } else if (speed == 0.75) {
            telemetry.addData("Speed", "Normal Boi");
        }
    }

    public void incrementThroughPositions() {

        telemetry.addData("Current Offset (by logic)", robot.sorterLogic.getCurrentOffset());
        //int newOffset = sorterLogic.getCurrentOffset();

        /*if (newOffset < 0) {
            return;
        }*/

        // This section of code increments by 1
        /*if (gamepad2.leftBumperWasPressed()) {
            targetOffset = makeSureNewOffsetIsOK(targetOffset - 1);
            robot.sorterHardware.prepareNewMovement(robot.sorterHardware.motor.getCurrentPosition(), sorterLogic.offsetPositions.get(targetOffset));
        }

        else if (gamepad2.rightBumperWasPressed()) {
            targetOffset = makeSureNewOffsetIsOK(targetOffset + 1);
            robot.sorterHardware.prepareNewMovement(robot.sorterHardware.motor.getCurrentPosition(), sorterLogic.offsetPositions.get(targetOffset));
        }*/

        // Fire positions
        if (gamepad2.dpadLeftWasPressed()) {
            goNextFirePosition(-1);
        } else if (gamepad2.dpadUpWasReleased()) {
            goNextFirePosition(1);
        }

        // Load positions
        else if (gamepad2.dpadDownWasPressed()){
            goNextLoadPosition(-1);
        } else if (gamepad2.dpadRightWasPressed()) {
            goNextLoadPosition(1);
        }

        telemetry.addData("Target Offset", targetOffset);
    }

    private void goNextLoadPosition(int go) {
        int potentialNewPosition = targetOffset + go;
        if (!isEven(potentialNewPosition)) {potentialNewPosition += go;}
        targetOffset = makeSureNewOffsetIsOK(potentialNewPosition);
        robot.sorterHardware.prepareNewMovement(robot.sorterHardware.motor.getCurrentPosition(), robot.sorterLogic.offsetPositions.get(targetOffset));
    }
    private void goNextFirePosition(int go) {
        int potentialNewPosition = targetOffset + go;
        if (isEven(potentialNewPosition)) {potentialNewPosition += go;}
        targetOffset = makeSureNewOffsetIsOK(potentialNewPosition);
        robot.sorterHardware.prepareNewMovement(robot.sorterHardware.motor.getCurrentPosition(), robot.sorterLogic.offsetPositions.get(targetOffset));
    }

    private int makeSureNewOffsetIsOK(int oldNewOffset) { // is this the cause of our lag
        while (oldNewOffset < 0) {
            oldNewOffset += 6;
        }
        while (oldNewOffset > 5) {
            oldNewOffset -= 6;
        }
        return oldNewOffset;
    }

    private void magicFixEverything() {
        if (gamepad2.shareWasPressed()) {
            robot.sorterHardware.onCooldown = false;
            robot.launcher.onCooldown = false;
            /// These might fix things but are untested
            robot.launcher.waitingToFire = false;
            //robot.sorterHardware.doorTarget = CLOSED;
            //robot.sorterHardware.wantToMoveDoor = true;

        }
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
        telemetry.addData("Current Load Slot", robot.sorterLogic.findCurrentSlotInPosition(LOAD).getName());
        telemetry.addData("Current Fire Slot", robot.sorterLogic.findCurrentSlotInPosition(FIRE).getName());
        telemetry.addData("Door Cooldown", robot.sorterHardware.cooldownTimer.seconds());
        telemetry.addData("Launcher Cooldown", robot.launcher.cooldownTimer);

        telemetry.addLine("Artifact Storage:");
        telemetry.addData("Total Inventory", robot.sorterLogic.inventory.getTotalCount());
        telemetry.addLine("Purple: " + robot.sorterLogic.inventory.getPurpleCount() +
                " Green: " + robot.sorterLogic.inventory.getGreenCount());
        telemetry.addData("Slot A", robot.sorterLogic.slotA.getOccupied());
        telemetry.addData("Slot B", robot.sorterLogic.slotB.getOccupied());
        telemetry.addData("Slot C", robot.sorterLogic.slotC.getOccupied());


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


