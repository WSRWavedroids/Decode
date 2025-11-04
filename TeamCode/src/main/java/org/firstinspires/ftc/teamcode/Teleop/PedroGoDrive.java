package org.firstinspires.ftc.teamcode.Teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.Robot;
import org.firstinspires.ftc.teamcode.Vision.WaveTag;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Configurable
@TeleOp(name = "Pedro Go Drive", group = "CompBot")
public class PedroGoDrive extends OpMode {
    public Robot robot = null;
    public double speed;


    private Follower follower;
    public Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;


    // Blackboard Keys
    public static final String ALLIANCE_KEY = "Alliance";
    public static final String PATTERN_KEY = "Pattern";
    public final String POSE_KEY = "Pose";
    public final String Target_KEY = "Target";

    public Limelight_Target_Scanner scanner = new Limelight_Target_Scanner();
    public WaveTag targetData = null;

    public double trackpadXMax = 1920;
    public double trackpadXMin = 0;
    public double trackpadYMax = 1020;
    public double trackpadYMin = 0;

    public double trackpadCurrentX;
    public double trackpadCurrentY;

    public double stickerOffsetX = -0.01;
    public double stickerOffsetY = 0.01;

    public Pose trackTarget;

    private Pose tagPosition;

    @Override
    public void init() {

        robot = new Robot(hardwareMap, telemetry, this);
        follower = Constants.createFollower(hardwareMap);
        if (blackboard.get(POSE_KEY) == null) {
            blackboard.put(POSE_KEY, new Pose(72,72,0));
        }
        follower.setStartingPose((Pose) blackboard.get(POSE_KEY));
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();


        //if using field centric youl need this lolzeez
        if (blackboard.get(ALLIANCE_KEY) == "BLUE") {
            scanner.InitLimeLightTargeting(2, robot.hardwareMap);
            tagPosition = (new Pose(12, 132, Math.toRadians(54)));

        } else if (blackboard.get(ALLIANCE_KEY) == "RED") {
            scanner.InitLimeLightTargeting(1, robot.hardwareMap);
            tagPosition = new Pose(132, 132, Math.toRadians(-54));

        } else {
            scanner.InitLimeLightTargeting(1, robot.hardwareMap);
            tagPosition = (enterStandardCoords(58.3464567, 55.6299213, Math.toRadians(-54)));
            //set tag pose

        }

        /*pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, (new Pose(0, 53))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build());*/
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
        gamepad1.setLedColor(0, 0, 255, 1000000000);
        gamepad2.setLedColor(0, 0, 255, 1000000000);
        speed = .5;
    }

    @Override
    public void loop() {
        //Call this once per loop
        targetData = scanner.tagInfo();
        follower.update();
        telemetryM.update();
        driveSpeed();


        if (targetData.currentlyDetected) {
            gamepad1.rumble(0.25, 0.25, 100);
            //gamepad1.rumble(100);
        }

        if(gamepad1.touchpad_finger_1)
        {
            trackpadCurrentX = gamepad1.touchpad_finger_1_x;
            trackpadCurrentY = gamepad1.touchpad_finger_1_y; // Corrected for inversion
            trackTarget = translateTrackpad(trackpadCurrentX, trackpadCurrentY, ""); // Sets tracktarget to coords

            telemetry.addData("Finger 1 x detected val: ", gamepad1.touchpad_finger_1_x);
            telemetry.addData("Finger 1 y detected val: ", gamepad1.touchpad_finger_1_y);

            telemetry.addData("Finger 1 x adjusted: ", trackpadCurrentX);
            telemetry.addData("Finger 1 y adjusted: ", trackpadCurrentY);

            telemetry.addData("Pedro Target Position: ", trackTarget);
        }
        else if(trackTarget == null)
        {
            trackTarget = new Pose(72, 72, 0);
        }


        telemetry.update();




        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp
            follower.setTeleOpDrive(
                    gamepad1.left_stick_x * speed,//swapped these for mason
                    -gamepad1.left_stick_y * speed,//swapped these for mason
                    -gamepad1.right_stick_x * speed,
                    false // Robot Centric
            );

        }

        //Automated PathFollowing
        //Mason's Pedro
        if (gamepad1.touchpadWasPressed()) {
            follower.followPath(makeDynamicPath(trackTarget, follower.getHeading()));
            automatedDrive = true;
            speed = 1;
        } else if (gamepad1.triangle) //Do a 180
        {
            follower.holdPoint(new BezierPoint(follower.getPose()), follower.getHeading() + Math.PI);
            automatedDrive = true;
            speed = 1;
        } else if (gamepad1.right_bumper)// Auto aim
        {
            follower.holdPoint(new BezierPoint(follower.getPose()), locateTagHeading(tagPosition, follower.getPose()));
            automatedDrive = true;
            speed = 1;
        }
        else if(gamepad1.left_bumper)
        {
            follower.holdPoint(new BezierPoint(follower.getPose()), follower.getHeading());

            //follower.followPath(makeDynamicPath(follower.getPose(), follower.getHeading()));
            automatedDrive = true;
            speed = 1;
        }



        //Stop automated following when the driver needs to
        if (automatedDrive && (gamepad1.bWasPressed())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        /*Slow Mode
        //if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        }

        //Optional way to change slow mode strength
        if (gamepad1.xWasPressed()) {
            slowModeMultiplier += 0.25;
        }*/



        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
    }

    private Pose enterStandardCoords(double x, double y, double heading) {
        return new Pose(x + 72, y + 72, heading);
    }

    private Pose translateTrackpad(double inX, double inY, String headingCheck)
    {


        //fix y axis inversion (top is 0 instead of bottom)
        //inY = Math.abs(inY - trackpadYMax);

        //if the heading check is tag rotate to point at target during path
        if (headingCheck == "tag")
        {
            return new Pose(((inX)*72)+72,((inY)*72)+72);
        }
        else //or just keep current heading for same movement
        {
            return new Pose(((inX)*72)+72,((inY)*72)+72);
        }
    }

    private double locateTagHeading(Pose tagPose, Pose robot)
    {
        double dx = tagPose.getX() - robot.getX();
        double dy = tagPose.getY() - robot.getY();
        return Math.atan2(dy, dx);
    }

    private PathChain makeDynamicPath(Pose targetPose, double targetHeadingRadians) {
        return follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), targetPose))
                .setLinearHeadingInterpolation(follower.getHeading(), targetHeadingRadians)
                .build();
                 // Build the PathChain after adding all paths
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




}
