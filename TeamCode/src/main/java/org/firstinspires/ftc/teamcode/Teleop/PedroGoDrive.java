package org.firstinspires.ftc.teamcode.Teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Configurable
@TeleOp(name = "Pedro Go Drive", group = "CompBot")
public class PedroGoDrive extends OpMode {
    public Robot robot = null;

    private Follower follower;
    public Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

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
        } else if (blackboard.get(ALLIANCE_KEY) == "RED") {
            scanner.InitLimeLightTargeting(1, robot.hardwareMap);
        } else {
            scanner.InitLimeLightTargeting(1, robot.hardwareMap);
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
    }

    @Override
    public void loop() {
        //Call this once per loop



        follower.update();
        telemetryM.update();

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false // Robot Centric
            );

                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    false // Robot Centric
            );
        }

        double touchX = gamepad1.touchpad_finger_1_x;
        double touchY =  gamepad1.touchpad_finger_1_y;

        //Automated PathFollowing
        if (gamepad1.triangle) {
            follower.followPath(makeDynamicPath(translateTrackpad(touchX, touchY, "") , follower.getHeading()));
            automatedDrive = true;
        }

        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        //Slow Mode
        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        }

        //Optional way to change slow mode strength
        if (gamepad1.xWasPressed()) {
            slowModeMultiplier += 0.25;
        }



        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
    }

    private Pose enterStandardCoords(double x, double y, double heading) {
        return new Pose(x + 72, y + 72, heading);
    }

    private Pose translateTrackpad(double inX, double inY, String headingCheck)
    {
        //So we don't drive off the field from misclicking
        if(inX > trackpadXMax)
        {
            inX = trackpadXMax;
        }
        else if(inX < trackpadXMin)
        {
            inX = trackpadXMin;
        }

        if(inY > trackpadYMax)
        {
            inY = trackpadYMax;
        }
        else if(inY < trackpadYMin)
        {
            inY = trackpadYMin;
        }

        //fix y axis inversion (top is 0 instead of bottom)
        //inY = Math.abs(inY - trackpadYMax);

        //if the heading check is tag rotate to point at target during path
        if (headingCheck == "tag")
        {
            return new Pose(((inX / trackpadXMax)*144),((inY / trackpadYMax)*144));
        }
        else //or just keep current heading for same movement
        {
            return new Pose(((inX / trackpadXMax)*144),((inY / trackpadYMax)*144));
        }



    }

    private PathChain makeDynamicPath(Pose targetPose, double targetHeadingRadians) {
        return follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), targetPose))
                .setLinearHeadingInterpolation(follower.getHeading(), follower.getHeading())
                .build();
                 // Build the PathChain after adding all paths
    }




}
