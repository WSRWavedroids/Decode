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

@Autonomous(name = "Blue Front 12 Ball", group = "Autonomous")
@Configurable // Panels
public class BlueFront12Ball extends OpMode {

    public Robot robot = null;
    public AutonomousPlusPLUS auto = null;
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private PathsForFront12Blue paths; // Paths defined in the Paths class

    private Timer pathTimer, actionTimer, opmodeTimer;

    @Override
    public void init() {

        robot = new Robot(hardwareMap, telemetry, this);
        auto = new AutonomousPlusPLUS(robot);

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(22.465, 123.866, Math.toRadians(55)));

        paths = new PathsForFront12Blue(follower, pathTimer); // Build paths

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

    public static class PathsForFront12Blue {

        public PathChain MoveAwayFromGoal;
        public double First3;
        public PathChain LineUpWithCenterBalls;
        public PathChain GrabCenterBalls;
        public PathChain MoveToLetBallsOut;
        public double letballsout;
        public PathChain MoveToScoreSecondPattern;
        public double Second3shots;
        public PathChain LineUpToGrabCloseBalls;
        public PathChain GrabCloseBalls;
        public PathChain MoveToFireThirdPattern;
        public double Thirdpattern;
        public PathChain LineUpWithFarBalls;
        public PathChain GrabFarBalls;
        public PathChain MoveForFinalPattern;
        public double Wait15;
        private int pathState;

        public PathsForFront12Blue(Follower follower, Timer pathTimer) {
            MoveAwayFromGoal = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(22.465, 123.866), new Pose(53.408, 112.845))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(55), Math.toRadians(145))
                    .build();


            LineUpWithCenterBalls = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(53.408, 112.845), new Pose(52.984, 58.378))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))
                    .build();

            GrabCenterBalls = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(52.984, 58.378), new Pose(8.477, 58.378))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

             MoveToLetBallsOut = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(8.477, 58.378),
                                    new Pose(36.665, 51.596),
                                    new Pose(16.107, 68.975)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                    .build();


            MoveToScoreSecondPattern = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(16.107, 68.975),
                                    new Pose(50.865, 65.584),
                                    new Pose(53.408, 112.845)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(145))
                    .build();


            LineUpToGrabCloseBalls = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(53.408, 112.845), new Pose(53.620, 84.658))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))
                    .build();

            GrabCloseBalls = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(53.620, 84.658), new Pose(18.650, 84.234))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            MoveToFireThirdPattern = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(18.650, 84.234), new Pose(53.408, 113.269))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))
                    .build();


            LineUpWithFarBalls = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(53.408, 113.269), new Pose(52.560, 35.065))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))
                    .build();

            GrabFarBalls = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(52.560, 35.065), new Pose(9.325, 34.641))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            MoveForFinalPattern = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(9.325, 34.641), new Pose(53.408, 113.481))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))
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
                follower.followPath(paths.MoveAwayFromGoal);
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
                    follower.followPath(paths.LineUpWithCenterBalls);
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
                    follower.followPath(paths.GrabCenterBalls);
                    setPathState(5);
                }

                break;
            case 5:
                if (!follower.isBusy()) {
                    //Disable Auto intake
                    follower.followPath(paths.MoveToLetBallsOut);
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
                    follower.followPath(paths.LineUpToGrabCloseBalls);
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
                    follower.followPath(paths.GrabCloseBalls);
                    setPathState(12);
                }
            case 12:
                if (!follower.isBusy()) {
                    follower.followPath(paths.MoveToFireThirdPattern);
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
                    follower.followPath(paths.GrabFarBalls);
                    setPathState(17);
                }
                break;
            case 17:
                if(!follower.isBusy())
                {
                    follower.followPath(paths.MoveForFinalPattern);
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
