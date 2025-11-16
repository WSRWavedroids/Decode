package org.firstinspires.ftc.teamcode.Core;

import static org.firstinspires.ftc.teamcode.Core.Robot.openClosed.CLOSED;

import android.annotation.SuppressLint;

import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

import java.util.Objects;


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


    public Panels panels;

    public static TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    public enum openClosed{OPEN,CLOSED}

    public boolean scanningForTargetTag = false;

    public int limelightSideOffsetAngle = 0;

    //Initialize motors and servos
    public Robot(HardwareMap hardwareMap, Telemetry telemetry, OpMode opmode){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.opmode = opmode;

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
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        sorterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeyServoL.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeyServoR.setDirection(DcMotorSimple.Direction.REVERSE);


        // This tells the motors to chill when we're not powering them.
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sorterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //This is new..
        telemetry.addData("Status", "Initialized");

        sorterHardware = new SorterHardware(this);
        launcher = new LauncherHardware(this);
        sorterLogic = new ArtifactLocator(this);
        inventoryCam = new SensorHuskyLens(this);
        targetScanner = new Limelight_Target_Scanner();
        randomizationScanner = new Limelight_Randomization_Scanner();


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

    public void setTargets(String direction, int ticks) {

        //This is all inverted (big sigh)

        if (Objects.equals(direction, "Right")){
            frontLeftDrive.setTargetPosition(-ticks + frontLeftDrive.getCurrentPosition());
            frontRightDrive.setTargetPosition(ticks + frontRightDrive.getCurrentPosition());
            backLeftDrive.setTargetPosition(ticks + backLeftDrive.getCurrentPosition());
            backRightDrive.setTargetPosition(-ticks + backRightDrive.getCurrentPosition());

        } else if (direction == "Left"){
            frontLeftDrive.setTargetPosition(ticks + frontLeftDrive.getCurrentPosition());
            frontRightDrive.setTargetPosition(-ticks + frontRightDrive.getCurrentPosition());
            backLeftDrive.setTargetPosition(-ticks + backLeftDrive.getCurrentPosition());
            backRightDrive.setTargetPosition(ticks + backRightDrive.getCurrentPosition());

        } else if (direction == "Forward"){
            frontLeftDrive.setTargetPosition(-ticks + frontLeftDrive.getCurrentPosition());
            frontRightDrive.setTargetPosition(-ticks + frontRightDrive.getCurrentPosition());
            backLeftDrive.setTargetPosition(-ticks + backLeftDrive.getCurrentPosition());
            backRightDrive.setTargetPosition(-ticks + backRightDrive.getCurrentPosition());

        } else if (direction == "Backward") {
            frontLeftDrive.setTargetPosition(ticks + frontLeftDrive.getCurrentPosition());
            frontRightDrive.setTargetPosition(ticks + frontRightDrive.getCurrentPosition());
            backLeftDrive.setTargetPosition(ticks + backLeftDrive.getCurrentPosition());
            backRightDrive.setTargetPosition(ticks + backRightDrive.getCurrentPosition());

        } else if (direction == "Turn Right") {
            frontLeftDrive.setTargetPosition(-ticks + frontLeftDrive.getCurrentPosition());
            frontRightDrive.setTargetPosition(ticks + frontRightDrive.getCurrentPosition());
            backLeftDrive.setTargetPosition(-ticks + backLeftDrive.getCurrentPosition());
            backRightDrive.setTargetPosition(ticks + backRightDrive.getCurrentPosition());

        } else if (direction == "Turn Left") {
            frontLeftDrive.setTargetPosition(ticks + frontLeftDrive.getCurrentPosition());
            frontRightDrive.setTargetPosition(-ticks + frontRightDrive.getCurrentPosition());
            backLeftDrive.setTargetPosition(ticks + backLeftDrive.getCurrentPosition());
            backRightDrive.setTargetPosition(-ticks + backRightDrive.getCurrentPosition());
        }
        else if (direction == "Diagonal Right") {
            frontLeftDrive.setTargetPosition(-ticks + frontLeftDrive.getCurrentPosition());
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setTargetPosition(-ticks + backRightDrive.getCurrentPosition());
        }
        else if (direction == "Diagonal Left") {
            frontLeftDrive.setPower(0);
            frontRightDrive.setTargetPosition(-ticks + frontRightDrive.getCurrentPosition());
            backLeftDrive.setTargetPosition(-ticks + backLeftDrive.getCurrentPosition());
            backRightDrive.setPower(0);
        }

    }

    public void positionRunningMode(){

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void powerRunningMode()
    {
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void powerSet(double speed) {
        frontLeftDrive.setPower(speed);
        frontRightDrive.setPower(speed);
        backLeftDrive.setPower(speed);
        backRightDrive.setPower(speed);

    }

    public DcMotor.RunMode encoderRunningMode(){
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        return null;
    }

    public void encoderReset(){
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

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

    public void runBasicIntake(double num)
    {
        intakeyServoR.setPower(num);
        intakeyServoL.setPower(num);
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
