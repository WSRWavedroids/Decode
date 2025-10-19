package org.firstinspires.ftc.teamcode.Teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;

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
            tagPosition = new Pose(58.3464567 + 72, 55.6299213 + 72, Math.toDegrees(54), PedroCoordinates.INSTANCE);
            //targetData.data = new Pose3D(new Position(DistanceUnit.INCH, 58.3464567 + 72, 55.6299213 + 72, 29.488189 + 72, 0),
                   //new YawPitchRollAngles(AngleUnit.DEGREES, 45, 0,0, 0));

        } else if (blackboard.get(ALLIANCE_KEY) == "RED") {
            scanner.InitLimeLightTargeting(1, robot.hardwareMap);
            tagPosition = new Pose(58.3464567 + 72,  -55.6299213 + 72, Math.toDegrees(-54), PedroCoordinates.INSTANCE);

            //targetData.data = new Pose3D(new Position(DistanceUnit.INCH, 58.3464567 + 72, -55.6299213 + 72, 29.488189 + 72, 0),
              //      new YawPitchRollAngles(AngleUnit.DEGREES, -45, 0,0, 0));
            //set the tag position
        } else {
            scanner.InitLimeLightTargeting(1, robot.hardwareMap);
            tagPosition = new Pose(58.3464567 + 72,  -55.6299213 + 72, Math.toDegrees(-54), PedroCoordinates.INSTANCE);
        }

        targetData = scanner.tagInfo();


    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
        gamepad1.setLedColor(0, 0, 255, 1000000000);
        gamepad2.setLedColor(0, 0, 255, 1000000000);

    }

    @Override
    public void loop() {
        //Call this once per loop

        if(targetData.currentlyDetected)
        {
            gamepad1.rumble(0.25, 0.25, 100);
        }


        if(gamepad1.touchpad_finger_1)
        {
            trackpadCurrentX = gamepad1.touchpad_finger_1_x;
            trackpadCurrentY = -gamepad1.touchpad_finger_1_y; // Corrected for inversion
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

        //Automated PathFollowing
        if (gamepad1.touchpadWasPressed()) {
            follower.followPath(makeDynamicPath(translateTrackpad(gamepad1.touchpad_finger_1_x, gamepad1.touchpad_finger_1_y, ""), findTagHeading()));
            automatedDrive = true;
        }

        if (gamepad1.cross) {
            follower.followPath(makeDynamicPath(follower.getPose(), findTagHeading()));
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



        telemetry.addData("position", follower.getPose());
        telemetry.addData("velocity", follower.getVelocity());
        telemetry.addData("automatedDrive", automatedDrive);
    }

    private Pose enterStandardCoords(double x, double y, double heading) {
        return new Pose(x + 72, y + 72, heading);
    }

    private Pose translateTrackpad(double inX, double inY, String headingCheck)
    {
            return new Pose(((inX)*72)+72,((inY)*72)+72);
    }

    private PathChain makeDynamicPath(Pose targetPose, double targetHeadingRadians) {
        return follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), targetPose))
                .setLinearHeadingInterpolation(follower.getHeading(), targetHeadingRadians)
                .build();
                 // Build the PathChain after adding all paths
    }

    private Pose getRobotPoseFromCamera()
    {

        if (targetData != null)
        {
            double x = targetData.robotFromTagPos.getPosition().x; // or whatever field coordinate data Limelight gives
            double y = targetData.robotFromTagPos.getPosition().y;
            double heading = targetData.robotFromTagPos.getOrientation().getYaw();

            // Convert FTC coords -> Pedro coords
            Pose pose = new Pose(x, y, heading, FTCCoordinates.INSTANCE);
            return pose.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
        }
        else
        {
            return null;
        }

    }

    public double findTagHeading()
    {
        Pose robotPose = follower.getPose();

        // Get difference between tag and robot
        double dx = tagPosition.getX() - robotPose.getX();
        double dy = tagPosition.getY() - robotPose.getY();

        // Get the angle (in radians) from robot to tag
        double targetHeading = Math.atan2(dy, dx) + (Math.PI);
        return targetHeading;
    }




}
