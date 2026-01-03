package org.firstinspires.ftc.teamcode.Autonomous.PEDRO;

import static org.firstinspires.ftc.teamcode.Core.ArtifactLocator.SlotState.*;
import static org.firstinspires.ftc.teamcode.Core.Robot.patternColors.*;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Autonomous.AutonomousPlusPLUS;
import org.firstinspires.ftc.teamcode.Core.ArtifactLocator;
import org.firstinspires.ftc.teamcode.Core.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red Back 12 Ball", group = "Autonomous")
@Configurable // Panels
public class RedBack12Ball extends OpMode {

    public Robot robot = null;
    public AutonomousPlusPLUS auto = null;
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private PathsForBack12Blue paths; // Paths defined in the Paths class

    private Timer pathTimer, actionTimer, opmodeTimer;

    @Override
    public void init() {

        robot = new Robot(hardwareMap, telemetry, this);
        auto = new AutonomousPlusPLUS(robot);

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(144-50.188, 9.200, Math.toRadians(90)));

        paths = new PathsForBack12Blue(follower, pathTimer); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    public void init_loop() {
        telemetry.addData("HYPE", "ARE! YOU! READY?!?!?!?!");

        robot.pattern = robot.randomizationScanner.GetRandomization();
        telemetry.addData(String.valueOf(robot.pattern), " Works!");
        telemetry.update();
    }

    /**
     * Code to run ONCE when the driver hits PLAY
     */
    public void start() {
        //runtime.reset();
        telemetry.addData("HYPE", "Let's do this!!!");
        robot.readyHardware(true);
        robot.sorterHardware.legalToSpin = true;
        //speed = 1;
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class PathsForBack12Blue {

        public PathChain MoveFromBackFiringZone;
        public PathChain LineUpWithMiddleBalls;
        public PathChain YOINKMIDDLE;
        public PathChain GoHitGate;
        public PathChain MoveToScoreSecondPattern;
        public PathChain LineUpWithClose;
        public PathChain YOINKFAR;
        public PathChain MoveToFireThridPattern;
        public PathChain LineUpWithFarBalls;
        public PathChain YOINKCLOSE;
        public PathChain ScoreFinalPattern;
        public PathChain Unpark;

        public PathsForBack12Blue(Follower follower, Timer pathTimer) {
            MoveFromBackFiringZone = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(144-50.188, 9.200), new Pose(144-57.689, 19.006))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(86.3105590062))
                    .build();

            LineUpWithMiddleBalls = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(144-57.689, 19.006), new Pose(144-47.859, 56.965))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(86.3105590062), Math.toRadians(0))
                    .build();

            YOINKMIDDLE = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(144-47.859, 56.965), new Pose(144-9.953, 57.176))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            GoHitGate = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(144-9.953, 57.176),
                                    new Pose(144-57.812, 63.529),
                                    new Pose(144-16.306, 68.824)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .build();

            MoveToScoreSecondPattern = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(144-16.306, 68.824),
                                    new Pose(144-54.212, 62.682),
                                    new Pose(144-57.600, 19.059)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(60))
                    .build();

            LineUpWithClose = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(144-57.600, 19.059), new Pose(144-44.471, 34.518))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(0))
                    .build();

            YOINKCLOSE = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(144-44.471, 34.518), new Pose(144-10.000, 35.365))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            MoveToFireThridPattern = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(144-10.000, 35.365), new Pose(144-57.812, 18.635))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(60))
                    .build();

            LineUpWithFarBalls = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(144-57.812, 18.635), new Pose(144-44.682, 83.859))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(0))
                    .build();

            YOINKCLOSE = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(144-44.682, 83.859), new Pose(144-17.000, 84.071))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            ScoreFinalPattern = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(144-17.000, 84.071), new Pose(144-56.329, 79.200))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(50))
                    .build();

            Unpark = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(144-56.329, 79.200), new Pose(144-56.541, 57.600))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(180))
                    .build();
        }
    }



    public int autonomousPathUpdate() {
        emergencyFinishIfNeeded();
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        switch (pathState) {
            case 0:
                follower.followPath(paths.MoveFromBackFiringZone);
                setPathState(1);
                break;
            case 1:
            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    auto.fireMatchPattern();
                    setPathState(2);
                }
                break;
            case 2:
                if (auto.fireInSequenceComplete()) {
                    follower.followPath(paths.LineUpWithMiddleBalls);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    //Enable auto Intake
                    actionTimer.resetTimer();
                    setPathState(4);
                }
                break;
            case 4:

                if(actionTimer.getElapsedTime() > 250)
                {
                    follower.followPath(paths.YOINKMIDDLE);
                    setPathState(5);
                }

                break;
            case 5:
                if (!follower.isBusy()) {
                    //Disable Auto intake
                    follower.followPath(paths.GoHitGate);
                    setPathState(6);
                }
                break;
            case 6: //Wait to let balls out
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(7);
                }
                break;
            case 7:
                if (actionTimer.getElapsedTime() > 500) {
                    follower.followPath(paths.MoveToScoreSecondPattern);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    auto.fireMatchPattern();
                    setPathState(9);
                }
                break;
            case 9:
                if (auto.fireInSequenceComplete()) {
                    follower.followPath(paths.LineUpWithClose);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    //Enable auto intake
                    actionTimer.resetTimer();
                    setPathState(11);
                }
                break;
            case 11:
                if (actionTimer.getElapsedTime() > 250) {
                    follower.followPath(paths.YOINKCLOSE);
                    setPathState(12);
                }
            case 12:
                if (!follower.isBusy()) {
                    follower.followPath(paths.MoveToFireThridPattern);
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    auto.fireMatchPattern();
                    setPathState(14);
                }
                break;
            case 14:
                if (auto.fireInSequenceComplete()) {
                    follower.followPath(paths.LineUpWithFarBalls);
                    setPathState(15);
                }
                break;
            case 15:
                if(!follower.isBusy())
                {
                    //activate intake
                    actionTimer.resetTimer();
                    setPathState(16);
                }
                break;
            case 16:
                if(actionTimer.getElapsedTime() > 0.25)
                {
                    follower.followPath(paths.YOINKFAR);
                    setPathState(17);
                }
                break;
            case 17:
                if(!follower.isBusy())
                {
                    follower.followPath(paths.ScoreFinalPattern);
                    setPathState(18);
                }
                break;
            case 18:
                if(!follower.isBusy())
                {
                    auto.fireMatchPattern();
                    setPathState(99);
                }
                break;


            case 99:
                return 0;

        }
        return 0;
    }
    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/

    void setPathState(int i) {
        {
            pathState = i;
            pathTimer.resetTimer();
        }

    }

    private void scan()
    {
        
    }

    private void emergencyFinishIfNeeded()
    {
        if(opmodeTimer.getElapsedTimeSeconds() > 29.5)
        {
            robot.sorterHardware.prepareNewMovement(0);
            setPathState(99);
        }
    }
}
