package org.firstinspires.ftc.teamcode.Core;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.*;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.*;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.*;
import static org.firstinspires.ftc.teamcode.Core.Robot.OpenClosed.*;

import android.annotation.SuppressLint;

import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Vision.Limelight_Target_Scanner;
import org.firstinspires.ftc.teamcode.Vision.SensorHuskyLens;
import org.firstinspires.ftc.teamcode.Vision.WaveTag;
import org.firstinspires.ftc.teamcode.Vision.Limelight_Randomization_Scanner;

public class Robot {

    public DcMotorEx frontLeftDrive;
    public DcMotorEx frontRightDrive;
    public DcMotorEx backLeftDrive;
    public DcMotorEx backRightDrive;

    public DcMotorEx sorterMotor;
    public DcMotorEx launcherMotor;

    public Servo hammerServo;
    public Servo doorServo;

    public CRServo expandyServo;

    public CRServo intakeyServoR;
    public CRServo intakeyServoL;

    public CRServo feedServoL;
    public CRServo feedServoR;

    public CRServo transferServoL;
    public CRServo transferServoR;

    public DigitalChannel magsense;

    public Limelight3A limelight;

   // public WebcamName CamCam;

    public HuskyLens husky;

    public Telemetry telemetry;
    //public BNO055IMU imu;

    //init and declare war
    public OpMode opmode;
    public HardwareMap hardwareMap;
    public String startingPosition;
    public String controlMode = "Robot Centric";// Robot Centric
    public IMU.Parameters imuParameters;
    public WaveTag targetTag = new WaveTag();
    public String pattern;

    public SorterHardware sorterHardware;
    public LauncherHardware launcher;
    public ArtifactLocator sorterLogic;
    public SensorHuskyLens inventoryCam;
    public Limelight_Randomization_Scanner randomizationScanner;
    public Limelight_Target_Scanner targetScanner;
    public fireQueue queue;


    public Panels panels;

    public static TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    public enum OpenClosed {OPEN,CLOSED}
    public enum CardinalDirections {
        FORWARD, BACKWARD, LEFT, RIGHT,
        DIAGONAL_LEFT, DIAGONAL_RIGHT,
        TURN_LEFT, TURN_RIGHT}

    public boolean scanningForTargetTag = false;

    public int limelightSideOffsetAngle = 0;

    //Initialize motors and servos
    public Robot(HardwareMap hardwareMap, Telemetry telemetry, OpMode opmode) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.opmode = opmode;

        // There's pizza here!

        // This section turns the names of the pieces of hardware into variables that we can program with.
        // Make sure that the device name is the exact same thing you typed in on the configuration on the driver hub.
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "frontRightDrive");
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "frontLeftDrive");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "backRightDrive");

        sorterMotor = hardwareMap.get(DcMotorEx.class, "sorterMotor");
        launcherMotor =  hardwareMap.get(DcMotorEx.class, "launcherMotor");

        hammerServo = hardwareMap.get(Servo.class, "hammerServo");
        doorServo = hardwareMap.get(Servo.class, "doorServo");

        expandyServo = hardwareMap.get(CRServo.class, "expandyServo");
        intakeyServoL = hardwareMap.get(CRServo.class, "intakeyServoL");
        intakeyServoR = hardwareMap.get(CRServo.class, "intakeyServoR");
        feedServoL = hardwareMap.get(CRServo.class, "feedServoL");
        feedServoR = hardwareMap.get(CRServo.class, "feedServoR");

        transferServoL = hardwareMap.get(CRServo.class, "transferServoL");
        transferServoR = hardwareMap.get(CRServo.class, "transferServoR");

        magsense = hardwareMap.get(DigitalChannel.class, "magsense");
        magsense.setMode(DigitalChannel.Mode.INPUT);

        //CamCam = hardwareMap.get(WebcamName.class, "CamCam");
        //expandyServo = hardwareMap.get(CRServo.class, "expandyServo");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        husky = hardwareMap.get(HuskyLens.class, "evenBetterMason");

        imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        );

        // This section sets the direction of all of the motors. Depending on the motor, this may change later in the program.
        frontLeftDrive.setDirection(FORWARD);
        frontRightDrive.setDirection(REVERSE);
        backLeftDrive.setDirection(FORWARD);
        backRightDrive.setDirection(REVERSE);

        sorterMotor.setDirection(REVERSE);

        intakeyServoL.setDirection(FORWARD);
        intakeyServoR.setDirection(REVERSE);

        transferServoL.setDirection(FORWARD);
        transferServoR.setDirection(REVERSE);

        // This tells the motors to chill when we're not powering them.
        frontRightDrive.setZeroPowerBehavior(BRAKE);
        backLeftDrive.setZeroPowerBehavior(BRAKE);
        backRightDrive.setZeroPowerBehavior(BRAKE);
        frontLeftDrive.setZeroPowerBehavior(BRAKE);
        sorterMotor.setZeroPowerBehavior(BRAKE);

        //This is new..
        telemetry.addData("Status", "Initialized");

        sorterHardware = new SorterHardware(this);
        launcher = new LauncherHardware(this);
        sorterLogic = new ArtifactLocator(this);
        inventoryCam = new SensorHuskyLens(this);
        queue = new fireQueue(this);
        targetScanner = new Limelight_Target_Scanner(this);
        randomizationScanner = new Limelight_Randomization_Scanner(this);
    }


    public boolean isWheelsBusy(){
        return backLeftDrive.isBusy() || frontLeftDrive.isBusy() || frontRightDrive.isBusy() || backRightDrive.isBusy();
    }

    public void stopAllMotors() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }

    /**
     * Runs the drive train in a cardinal direction.
     * @param direction The direction, a cardinalDirection enum
     * @param ticks The distance to move in motor ticks
     */
    public void setTargets(CardinalDirections direction, int ticks) {

        // This is all inverted (big sigh)

        switch (direction) {
            case RIGHT:
                frontLeftDrive.setTargetPosition(-ticks + frontLeftDrive.getCurrentPosition());
                frontRightDrive.setTargetPosition(ticks + frontRightDrive.getCurrentPosition());
                backLeftDrive.setTargetPosition(ticks + backLeftDrive.getCurrentPosition());
                backRightDrive.setTargetPosition(-ticks + backRightDrive.getCurrentPosition());
                break;
            case LEFT:
                frontLeftDrive.setTargetPosition(ticks + frontLeftDrive.getCurrentPosition());
                frontRightDrive.setTargetPosition(-ticks + frontRightDrive.getCurrentPosition());
                backLeftDrive.setTargetPosition(-ticks + backLeftDrive.getCurrentPosition());
                backRightDrive.setTargetPosition(ticks + backRightDrive.getCurrentPosition());
                break;
            case FORWARD:
                frontLeftDrive.setTargetPosition(-ticks + frontLeftDrive.getCurrentPosition());
                frontRightDrive.setTargetPosition(-ticks + frontRightDrive.getCurrentPosition());
                backLeftDrive.setTargetPosition(-ticks + backLeftDrive.getCurrentPosition());
                backRightDrive.setTargetPosition(-ticks + backRightDrive.getCurrentPosition());
                break;
            case BACKWARD:
                frontLeftDrive.setTargetPosition(ticks + frontLeftDrive.getCurrentPosition());
                frontRightDrive.setTargetPosition(ticks + frontRightDrive.getCurrentPosition());
                backLeftDrive.setTargetPosition(ticks + backLeftDrive.getCurrentPosition());
                backRightDrive.setTargetPosition(ticks + backRightDrive.getCurrentPosition());
                break;
            case TURN_RIGHT:
                frontLeftDrive.setTargetPosition(-ticks + frontLeftDrive.getCurrentPosition());
                frontRightDrive.setTargetPosition(ticks + frontRightDrive.getCurrentPosition());
                backLeftDrive.setTargetPosition(-ticks + backLeftDrive.getCurrentPosition());
                backRightDrive.setTargetPosition(ticks + backRightDrive.getCurrentPosition());
                break;
            case TURN_LEFT:
                frontLeftDrive.setTargetPosition(ticks + frontLeftDrive.getCurrentPosition());
                frontRightDrive.setTargetPosition(-ticks + frontRightDrive.getCurrentPosition());
                backLeftDrive.setTargetPosition(ticks + backLeftDrive.getCurrentPosition());
                backRightDrive.setTargetPosition(-ticks + backRightDrive.getCurrentPosition());
                break;
            case DIAGONAL_RIGHT:
                frontLeftDrive.setTargetPosition(-ticks + frontLeftDrive.getCurrentPosition());
                frontRightDrive.setPower(0);
                backLeftDrive.setPower(0);
                backRightDrive.setTargetPosition(-ticks + backRightDrive.getCurrentPosition());
                break;
            case DIAGONAL_LEFT:
                frontLeftDrive.setPower(0);
                frontRightDrive.setTargetPosition(-ticks + frontRightDrive.getCurrentPosition());
                backLeftDrive.setTargetPosition(-ticks + backLeftDrive.getCurrentPosition());
                backRightDrive.setPower(0);
                break;
        }

    }

    /**
     * Sets the drive motors to RUN_TO_POSITION. Enables usage of the DcMotor.setTargetPosition()
     * and Robot.setTargets() functions.
     */
    public void positionRunningMode() {

        frontLeftDrive.setMode(RUN_TO_POSITION);
        frontRightDrive.setMode(RUN_TO_POSITION);
        backLeftDrive.setMode(RUN_TO_POSITION);
        backRightDrive.setMode(RUN_TO_POSITION);
    }

    /**
     * Turns off the motor encoders, to run purely on power.
     */
    public void powerRunningMode()
    {
        frontLeftDrive.setMode(RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(RUN_WITHOUT_ENCODER);
    }
    public void powerSet(double speed) {
        frontLeftDrive.setPower(speed);
        frontRightDrive.setPower(speed);
        backLeftDrive.setPower(speed);
        backRightDrive.setPower(speed);

    }

    /**
     * Sets the motors to run with encoder feedback.
     */
    public void encoderRunningMode(){
        frontLeftDrive.setMode(RUN_USING_ENCODER);
        frontRightDrive.setMode(RUN_USING_ENCODER);
        backLeftDrive.setMode(RUN_USING_ENCODER);
        backRightDrive.setMode(RUN_USING_ENCODER);
    }

    public void encoderReset(){
        frontLeftDrive.setMode(STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(STOP_AND_RESET_ENCODER);
    }

    /**
     * Adds motor data to telemetry and updates it.
     */
    @SuppressLint("DefaultLocale")
    public void tellMotorOutput(){
        telemetry.addData("Control Mode", controlMode);
        telemetry.addData("Motors", String.format("FL Power(%.2f) FL Location (%d) FL Target (%d)", frontLeftDrive.getPower(), frontLeftDrive.getCurrentPosition(), frontLeftDrive.getTargetPosition()));
        telemetry.addData("Motors", String.format("FR Power(%.2f) FR Location (%d) FR Target (%d)", frontRightDrive.getPower(), frontRightDrive.getCurrentPosition(), frontRightDrive.getTargetPosition()));
        telemetry.addData("Motors", String.format("BL Power(%.2f) BL Location (%d) BL Target (%d)", backLeftDrive.getPower(), backLeftDrive.getCurrentPosition(), backLeftDrive.getTargetPosition()));
        telemetry.addData("Motors", String.format("BR Power(%.2f) BR Location (%d) BR Target (%d)", backRightDrive.getPower(), backRightDrive.getCurrentPosition(), backRightDrive.getTargetPosition()));

        telemetry.update();
    }

    public double inchesToTicks(double inches) {
        // returns the inches * ticks per rotation / wheel circ
        return ((inches/12.25) * 537.6 / .5);
        //todo Reference that 1 inch ~= 50 ticks
    }

    ElapsedTime timer = new ElapsedTime();


    public void prepareAuto(){
        sorterHardware.triggerServo(CLOSED);
        launcher.hammerServo.setPosition(launcher.hammerBackPosition);
    }

    /**
     * Updates the SorterHardware, LauncherHardware, ArtifactLocator, Limelight, and HuskyLens.
     * Also adds some data to telemetry.
     */
    public void updateAllDaThings()
    {
        sorterHardware.updateSorterHardware();
        launcher.updateLauncherHardware();
        sorterLogic.update();
        sorterHardware.updateSorterHardware();
        launcher.updateLauncherHardware();

        if(scanningForTargetTag)
        {
            targetTag = targetScanner.tagInfo();
        }


        panelsTelemetry.update();
        inventoryCam.updateBlockScan();



        dumpAllTelemetryFromUpdate();
    }

    public void dumpAllTelemetryFromUpdate()
    {
        //Reliant functions not present
        telemetry.addData("Sorter Position: ", sorterHardware.motor.getCurrentPosition());
        telemetry.addData("Reference", sorterHardware.motor.getCurrentPosition());
        telemetry.addData("Launcher Velocity: ", launcher.motor.getVelocity());
        telemetry.addData("DOOR: ", sorterHardware.doorTarget);
        telemetry.addData("Sorter In Position", sorterHardware.positionedCheck());
        telemetry.addData("Sorter State: ", sorterLogic.getCurrentOffset());
        telemetry.addData("Limelight angleX: ", targetTag.angleX);

    }

    /**
     * Sets the intake and feeder servos to run.
     * @param num The power input, from -1.0 to 1.0.
     */
    public void runBasicIntake(double num)
    {
        intakeyServoR.setPower(num);
        intakeyServoL.setPower(num);
        transferServoR.setPower(num);
        transferServoL.setPower(num);
    }

    public void runAutoIntakeSequence() //Run in an update function for "fast" auto load
    {
        //Find first empty
        runBasicIntake(1);
        sorterHardware.tryingToFeed = true;
        //sorterHardware.prepareNewMovement(sorterHardware.motor.getCurrentPosition(), sorterLogic.findFirstType(EMPTY).getLoadPosition());/*replace with first empty*/
        sorterHardware.feederIntakeSpeed = 1;
    }

    public void cancelAutoIntake()
    {
        sorterHardware.feederIntakeSpeed = 0;
        sorterHardware.tryingToFeed = false;
        runBasicIntake(0);
    }

    public void readyHardware(boolean resetEncoder)
    {

        launcher.hammerServo.setPosition(launcher.hammerBackPosition);
        sorterHardware.doorServo.setPosition(sorterHardware.doorClosedPosition);
        sorterHardware.alreadyClosed = true;
        sorterHardware.doorTarget  = CLOSED;
        launcher.setLauncherSpeed(0);
        inventoryCam.updateBlockScan();

        if(resetEncoder)
        {
            sorterHardware.resetSorterEncoder();
            encoderReset();
            sorterHardware.reference = 0;
        }
    }
}
