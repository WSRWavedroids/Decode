package org.firstinspires.ftc.teamcode.Teleop;

import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Core.ArtifactLocator;
import org.firstinspires.ftc.teamcode.Core.LauncherHardware;
import org.firstinspires.ftc.teamcode.Core.Robot;
import org.firstinspires.ftc.teamcode.Core.SorterHardware;
import org.firstinspires.ftc.teamcode.Vision.SensorHuskyLens;
import org.firstinspires.ftc.teamcode.Vision.WaveTag;

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
@TeleOp(name = "Basic Vortex", group = "CompBot")
public class Vortex_Teleop_Decode extends OpMode {

    // This section tells the program all of the different pieces of hardware that are on our robot that we will use in the program.
    private ElapsedTime runtime = new ElapsedTime();
    private double speed = 0.75;
    private boolean spinTargetAcquired = false;

    private boolean cadenRecording = false;
    private boolean contTwoBumpersPressed = false;

    int SpinTargetFrontLeft;
    int SpinTargetFrontRight;
    int SpinTargetBackLeft;
    int SpinTargetBackRight;

    int slot = 0; // temp for testing lol

    int targetOffset = 0;

    //private double storedSpeed;
    public Robot robot = null;
    public IMU imu;
    public Limelight_Target_Scanner tagScanner;
    public WaveTag targetData = null;

    //public SensorHuskyLens huskyLens;

    public SorterHardware sorterHardware;
    public LauncherHardware launcher;
    public ArtifactLocator sorterLogic;
    public SensorHuskyLens huskyLens;

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
        tagScanner = robot.targetScanner;
        sorterHardware = robot.sorterHardware;
        huskyLens = robot.inventoryCam;
        launcher = robot.launcher;
        sorterLogic = robot.sorterLogic;

        sorterHardware.doorServo.setPosition(sorterHardware.doorClosedPosition);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        outtakeTimer.reset();

        if (robot.controlMode == "Field Centric") {

            imu = hardwareMap.get(IMU.class, "imu");
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)); //Forward = left fsr
            // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
            imu.initialize(parameters);
        }
        //if using field centric youl need this lolzeez
        if (blackboard.get(ALLIANCE_KEY) == "BLUE") {
            tagScanner.InitLimeLightTargeting(2, robot.hardwareMap);
        } else if (blackboard.get(ALLIANCE_KEY) == "RED") {
            tagScanner.InitLimeLightTargeting(1, robot.hardwareMap);
        } else {
            tagScanner.InitLimeLightTargeting(1, robot.hardwareMap);
        }

        robot.panels = Panels.INSTANCE;
        robot.readyHardware();
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
        sorterHardware.resetSorterEncoder();//REMOVE ONCE AUTO -> TELE IS FIGURED OUT
        sorterHardware.legalToSpin = true;
        robot.readyHardware();

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


        if(gamepad2.square)
        {
            robot.expandyServo.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.expandyServo.setPower(1);

            /*Bot.frontLeftDrive.setPower(.1);
            Bot.frontRightDrive.setPower(.1);
            Bot.backLeftDrive.setPower(-0.1);
            Bot.backRightDrive.setPower(-0.1);*/


        }
        else if(gamepad2.triangle)
        {
            robot.expandyServo.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.expandyServo.setPower(1);

            //Drivetrain Assist?
            /*
            Bot.frontLeftDrive.setPower(-.1);
            Bot.frontRightDrive.setPower(-.1);
            Bot.backLeftDrive.setPower(0.1);
            Bot.backRightDrive.setPower(0.1);*/

        }
        else
        {
            robot.expandyServo.setPower(0);
        }

        if(gamepad2.cross)
        {
            //robot.runBasicIntake(1);
            robot.runAutoIntakeSequence();
        }
        else if(gamepad2.circle)
        {
            //robot.runBasicIntake(-1);
            robot.runAutoIntakeSequence();
        }
        else
        {
           robot.cancelAutoIntake();
        }

         // temp measure
        //testing rotation

        /*if (gamepad2.dpad_down)
        {
            sorterHardware.prepareNewMovement(robot.sorterMotor.getCurrentPosition(), sorterHardware.positions[slot]);
        }
        else if (gamepad2.dpad_up)
        {
            int nextSlot = (slot + 1) % sorterHardware.positions.length;
            sorterHardware.prepareNewMovement(robot.sorterMotor.getCurrentPosition(), sorterHardware.positions[nextSlot]);
        }*/

        incrementThroughPositions();

        /*else if (gamepad2.dpad_left & !contTwoBumpersPressed)
        {
            slot -= 2;
            if (slot < 0) slot = 4; // wrap back to last slot pair
            sorterHardware.prepareNewMovement(robot.sorterMotor.getCurrentPosition(), sorterHardware.positions[slot]);
            contTwoBumpersPressed = true;
        }
        else if (gamepad2.dpad_right & !contTwoBumpersPressed)
        {
            slot += 2;
            if (slot > 4) slot = 0; // wrap to first slot pair
            sorterHardware.prepareNewMovement(robot.sorterMotor.getCurrentPosition(), sorterHardware.positions[slot]);
            contTwoBumpersPressed = true;
        }
        else if (!gamepad2.dpad_left & !gamepad2.dpad_right) {
            contTwoBumpersPressed = false;
        }*/
        telemetry.addData("currentSlot target: ", slot);

        if (gamepad1.touchpad || gamepad2.touchpad) {
            requestOpModeStop();
        }

        //robot.panelsTelemetry.addData("Motor Position", robot.launcher.motor.getCurrentPosition());
        robot.panelsTelemetry.update();

        doTelemetryStuff();

        if(gamepad2.share)
        {
            cadenRecording = true;
        }
        if(gamepad2.dpad_down)//cancel and clear
        {
            cadenRecording = false;
            cadenFirstOccupied = false;
            cadenSecondOccupied = false;
            cadenThirdOccupied = false;
            first = 0;
            second = 0;
            third = 0;
        }

        //Prepare any
        if(gamepad2.left_trigger > 0.5)
        {
            if(cadenRecording)
            {
                //add new to current recording
                //addToCadenRecording(sorterLogic.findFirstOccupied());
            }
            else
            {
                sorterHardware.prepareNewMovement(robot.sorterHardware.motor.getCurrentPosition(),
                        (int) robot.sorterLogic.findFirstNotType(ArtifactLocator.slotState.EMPTY).getFirePosition());
            }
            //Find first occupied and prepare acion
        }
        //prepare green
        else if(gamepad2.triangle)
        {
            //Find first green and prepare action
            if(cadenRecording)
            {
                //addToCadenRecording(sorterLogic.findFirstOfType("Green"));
            }
            else
            {
                sorterHardware.prepareNewMovement(robot.sorterHardware.motor.getCurrentPosition(),
                        (int) robot.sorterLogic.findFirstType(ArtifactLocator.slotState.GREEN).getFirePosition());
            }
        }
        //prepare purple
        else if(gamepad2.square)
        {
            robot.runAutoIntakeSequence();
            /*if(cadenRecording)
            {

            }
            else
            {
                sorterHardware.prepareNewMovement(robot.sorterHardware.motor.getCurrentPosition(),
                        (int) robot.sorterLogic.findFirstType(ArtifactLocator.slotState.PURPLE).getFirePosition());
            }*/
        }

        if(gamepad2.right_trigger > 0.5)
        {
            launcher.readyFire();
        }

        /*if(gamepad2.dpad_down)
        {
           sorterHardware.estop;
        }*/

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
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
            double turnTicks = robot.targetTag.angleX * constant;

            robot.frontLeftDrive.setTargetPosition(robot.frontLeftDrive.getCurrentPosition() + (int) turnTicks);
            robot.frontRightDrive.setTargetPosition(robot.frontRightDrive.getCurrentPosition() - (int) turnTicks);
            robot.backLeftDrive.setTargetPosition(robot.backLeftDrive.getCurrentPosition() + (int) turnTicks);
            robot.backRightDrive.setTargetPosition(robot.backRightDrive.getCurrentPosition() - (int) turnTicks);
        }


        robot.frontLeftDrive.setPower(speed);
        robot.frontRightDrive.setPower(speed);
        robot.backLeftDrive.setPower(speed);
        robot.backRightDrive.setPower(speed);

        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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

    boolean cadenFirstOccupied = false;
    boolean cadenSecondOccupied = false;
    boolean cadenThirdOccupied = false;
    int first;
    int second;
    int third;
    private void addToCadenRecording(int physicalTargetTicks)
    {
        if(!cadenFirstOccupied)
        {
            first = physicalTargetTicks;
            cadenFirstOccupied = true;
        }
        else if(cadenFirstOccupied && !cadenSecondOccupied)
        {
            second = physicalTargetTicks;
            cadenSecondOccupied = true;
        }
        else if(cadenSecondOccupied && !cadenThirdOccupied)
        {
            third = physicalTargetTicks;
            cadenThirdOccupied = true;
            cadenRecording = false;
        }

    }

    public void incrementThroughPositions() {

        telemetry.addData("Current Offset (by logic)", sorterLogic.getCurrentOffset());
        //int newOffset = sorterLogic.getCurrentOffset();

        /*if (newOffset < 0) {
            return;
        }*/

        if (gamepad2.leftBumperWasPressed()) {
            targetOffset = makeSureNewOffsetIsOK(targetOffset - 1);
            sorterHardware.prepareNewMovement(sorterHardware.motor.getCurrentPosition(), sorterLogic.offsetPositions.get(targetOffset));
        }

        else if (gamepad2.rightBumperWasPressed()) {
            targetOffset = makeSureNewOffsetIsOK(targetOffset + 1);
            sorterHardware.prepareNewMovement(sorterHardware.motor.getCurrentPosition(), sorterLogic.offsetPositions.get(targetOffset));
        }

        telemetry.addData("Target Offset", targetOffset);
    }

    private int makeSureNewOffsetIsOK(int oldNewOffset) {
        if (oldNewOffset < 0) {
            oldNewOffset = 5;
        }
        if (oldNewOffset > 5) {
            oldNewOffset = 0;
        }
        return oldNewOffset;
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
        telemetry.addData("Equalized Target Position", sorterLogic.offsetPositions.get(targetOffset));

        //robot.tellMotorOutput();
    }
    private float getLargestAbsVal( float[] values){
        // This function does some math!
        float max = 0;
        for (float val : values) {
            if (Math.abs(val) > max) {
                max = Math.abs(val);
            }
        }
        return max;
    }



}


