package org.firstinspires.ftc.teamcode.Autonomous.PEDRO;

import static org.firstinspires.ftc.teamcode.Core.ArtifactLocator.SlotState.*;
import static org.firstinspires.ftc.teamcode.Core.Robot.allianceSides.BLUE;
import static org.firstinspires.ftc.teamcode.Core.Robot.patternColors.*;
import static org.firstinspires.ftc.teamcode.Autonomous.PEDRO.BlueBack12Ball.Steps.*;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.AutonomousPlusPLUS;
import org.firstinspires.ftc.teamcode.Core.Robot;
import org.firstinspires.ftc.teamcode.Core.TurretLogic;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue Back 12 Ball", group = "1. FABIO WITH PEDRO")
@Configurable // Panels
public class BlueBack12Ball extends OpMode {

    public Robot robot = null;
    public AutonomousPlusPLUS auto = null;
    TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    int pathState; // Current autonomous path state (state machine)
    PathsForBack12Blue paths; // Paths defined in the Paths class

    public static final String ALLIANCE_KEY = "Alliance"; //For blackboard
    public static final String PATTERN_KEY = "Pattern";
    public ElapsedTime stallTimer;
    Pose startPose = new Pose(50.188, 9.200, Math.PI / 2); // Make sure this is set HERE
    Timer pathTimer, actionTimer, opmodeTimer;

    @Override
    public void init() {

        robot = new Robot(hardwareMap, telemetry, this);
        TurretLogic.tolerance = TurretLogic.degreesToTicks(8);
        follower = robot.turret.follower;
        follower.setMaxPowerScaling(1);
        follower.setMaxPower(1);
        panelsTelemetry = Robot.panelsTelemetry;
        opmodeTimer = new Timer();
        pathTimer = new Timer();
        actionTimer = new Timer();
//        telemetry.addData("tolerance value test pt 1", TurretLogic.tolerance);
//        telemetry.addData("tolerance value test pt 2", TurretLogic.tolerance);
        auto = new AutonomousPlusPLUS(robot);
        robot.turret.activeMode = TurretLogic.controlMode.FULL;

//        robot.randomizationScanner.InitLimeLight(0);
//        blackboard.put(ALLIANCE_KEY, "BLUE");
//        stallTimer = new ElapsedTime();
//
//        robot.turret.follower.setPose(startPose);
//        robot.turret.follower.setHeading(startPose.getHeading());
//
//        robot.alliance = BLUE;

        paths = new PathsForBack12Blue(follower); // Build paths

//        panelsTelemetry.debug("Status", "Initialized");
//        panelsTelemetry.update(telemetry);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    public void init_loop() {
//        robot.pattern = robot.randomizationScanner.GetRandomization();
//        telemetry.addData(String.valueOf(robot.pattern), " Works!");
//        telemetry.update();
//
//        telemetry.addData("Our pattern is: ", String.valueOf(robot.pattern), " ...yay");
//
//        switch (robot.pattern) {
//            case PPG:
//                telemetry.addData("We doin", " PPG now");
//                blackboard.put(PATTERN_KEY, "PPG");
//                break;
//            case GPP:
//                telemetry.addData("We doin", " GPP now");
//                blackboard.put(PATTERN_KEY, "GPP");
//                break;
//            case PGP:
//                telemetry.addData("We doin", " PGP now");
//                blackboard.put(PATTERN_KEY, "PGP");
//                break;
//            default:
//                telemetry.addData("It failed ", "cry time");
//                break;
//        }
    }

    /**
     * Code to run ONCE when the driver hits PLAY
     */
    public void start() {
        //runtime.reset();
        opmodeTimer.resetTimer();
        telemetry.addData("HYPE", "Let's do this!!!");
        robot.readyHardware(false);
//        robot.sorterHardware.legalToSpin = true;
        //speed = 1;
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        robot.updateAllDaThings();
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
//        panelsTelemetry.debug("Path State", currentStep);
//        panelsTelemetry.debug("X", follower.getPose().getX());
//        panelsTelemetry.debug("Y", follower.getPose().getY());
//        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
//        panelsTelemetry.debug("Max Power Scalar", follower.getMaxPowerScaling());
//        panelsTelemetry.debug("Motor power", robot.frontLeftDrive.getVelocity());
//        panelsTelemetry.update(telemetry);
    }

    public static class PathsForBack12Blue {

        public PathChain MoveFromBackFiringZone;
        public PathChain LineUpWithMiddleBalls;
        public PathChain YOINKMIDDLE;
        public PathChain GoHitGate;
        public PathChain MoveToScoreSecondPattern;
        public PathChain MoveToScoreSecondPatternNoGate;
        public PathChain LineUpWithClose;
        public PathChain YOINKFAR;
        public PathChain MoveToFireThirdPattern;
        public PathChain LineUpWithFarBalls;
        public PathChain YOINKCLOSE;
        public PathChain ScoreFinalPattern;
        public PathChain Unpark;

        public PathsForBack12Blue(Follower follower) {
            MoveFromBackFiringZone = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(50.188, 9.200),
                                    new Pose(57.689, 19.006)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(113))
                    .build();

            LineUpWithMiddleBalls = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(57.689, 19.006), new Pose(47.859, 56.965))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(180))
                    .build();

            YOINKMIDDLE = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(47.859, 56.965), new Pose(9.953, 57.176))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            GoHitGate = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(9.953, 57.176),
                                    new Pose(57.812, 63.529),
                                    new Pose(16.306, 68.824)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                    .build();

            MoveToScoreSecondPattern = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(16.306, 68.824),
                                    new Pose(54.212, 62.682),
                                    new Pose(57.600, 19.059)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(113))
                    .build();

            MoveToScoreSecondPatternNoGate = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(9.953, 57.176),
                                    new Pose(54.212, 62.682),
                                    new Pose(57.600, 19.059)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(113))
                    .build();

            LineUpWithClose = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(57.600, 19.059), new Pose(44.471, 34.518))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(180))
                    .build();

            YOINKCLOSE = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(44.471, 34.518), new Pose(10.000, 35.365))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            MoveToFireThirdPattern = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(10.000, 35.365), new Pose(57.812, 18.635))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(113))
                    .build();

            LineUpWithFarBalls = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(57.812, 18.635), new Pose(44.682, 83.859))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(180))
                    .build();

            YOINKCLOSE = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(44.682, 83.859), new Pose(17.000, 84.071))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            ScoreFinalPattern = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(17.000, 84.071), new Pose(56.329, 79.200))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
                    .build();

            Unpark = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.329, 79.200), new Pose(56.541, 57.600))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(0))
                    .build();
        }
    }

    public enum Steps {
        START,
        MOVE_TO_FIRE_1, FIRE_1,
        LINE_UP_2, ENABLE_INTAKE_2, YOINK_2,
        HIT_GATE, WAIT_FOR_GATE_EMPTY,
        MOVE_TO_FIRE_2, FIRE_2,
        LINE_UP_3, ENABLE_INTAKE_3, YOINK_3, MOVE_TO_FIRE_3, FIRE_3,
        LINE_UP_4, ENABLE_INTAKE_4, YOINK_4, MOVE_TO_FIRE_4, FIRE_4,
        END
    }


    private Steps currentStep = START; // Current autonomous path state (state machine)

    public int autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine

        emergencyFinishIfNeeded();

        switch (currentStep) {
            case START:
                setCurrentStep(MOVE_TO_FIRE_1);
                break;
            case MOVE_TO_FIRE_1:
                follower.followPath(paths.MoveFromBackFiringZone);
                setCurrentStep(FIRE_1);
                break;
            case FIRE_1:
            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            *
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    robot.queue.addPattern(robot.pattern);
                    setCurrentStep(LINE_UP_2);
                }
                break;
            case LINE_UP_2:
                if (/*robot.queue.noBallsQueued*/ true) {
                    follower.followPath(paths.LineUpWithMiddleBalls);
                    setCurrentStep(ENABLE_INTAKE_2);
                }
                break;
            case ENABLE_INTAKE_2:
                if (!follower.isBusy()) {
                    //Enable auto Intake
                    actionTimer.resetTimer();
                    setCurrentStep(YOINK_2);
                }
                break;
            case YOINK_2:
                if(actionTimer.getElapsedTime() > 250)
                {
                    follower.followPath(paths.YOINKMIDDLE);
                    setCurrentStep(MOVE_TO_FIRE_2);
                }

                break;
            case HIT_GATE:
                if (!follower.isBusy()) {
                    //Disable Auto intake
                    follower.followPath(paths.GoHitGate);
                    setCurrentStep(WAIT_FOR_GATE_EMPTY);
                }
                break;
            case WAIT_FOR_GATE_EMPTY: //Wait to let balls out
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setCurrentStep(MOVE_TO_FIRE_2);
                }
                break;
            case MOVE_TO_FIRE_2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.MoveToScoreSecondPatternNoGate);
                    setCurrentStep(FIRE_2);
                }
                break;
            case FIRE_2:
                if (!follower.isBusy()) {
//                    auto.fireMatchPattern();
                    setCurrentStep(LINE_UP_3);
                }
                break;
            case LINE_UP_3:
                if (/*auto.fireInSequenceComplete()*/ true) {
                    follower.followPath(paths.LineUpWithClose);
                    setCurrentStep(ENABLE_INTAKE_3);
                }
                break;
            case ENABLE_INTAKE_3:
                if (!follower.isBusy()) {
                    //Enable auto intake
                    actionTimer.resetTimer();
                    setCurrentStep(YOINK_3);
                }
                break;
            case YOINK_3:
                if (actionTimer.getElapsedTime() > 250) {
                    follower.followPath(paths.YOINKCLOSE);
                    setCurrentStep(MOVE_TO_FIRE_3);
                }
            case MOVE_TO_FIRE_3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.MoveToFireThirdPattern);
                    setCurrentStep(FIRE_3);
                }
                break;
            case FIRE_3:
                if (!follower.isBusy()) {
//                    auto.fireMatchPattern();
                    setCurrentStep(LINE_UP_4);
                }
                break;
            case LINE_UP_4:
                if (/*auto.fireInSequenceComplete()*/ true) {
                    follower.followPath(paths.LineUpWithFarBalls);
                    setCurrentStep(ENABLE_INTAKE_4);
                }
                break;
            case ENABLE_INTAKE_4:
                if(!follower.isBusy())
                {
                    //activate intake
                    actionTimer.resetTimer();
                    setCurrentStep(YOINK_4);
                }
                break;
            case YOINK_4:
                if(actionTimer.getElapsedTime() > 0.25)
                {
                    follower.followPath(paths.YOINKFAR);
                    setCurrentStep(MOVE_TO_FIRE_4);
                }
                break;
            case MOVE_TO_FIRE_4:
                if(!follower.isBusy())
                {
                    follower.followPath(paths.ScoreFinalPattern);
                    setCurrentStep(FIRE_4);
                }
                break;
            case FIRE_4:
                if(!follower.isBusy())
                {
//                    auto.fireMatchPattern();
//                    setCurrentStep(END);
                }
                break;
            case END:
                return 0;
        }
        return 0;
    }

    /** These change the states of the paths and actions.
     * It will also reset the timers of the individual switches **/

    void setCurrentStep(Steps nextStep) {
        currentStep = nextStep;
        pathTimer.resetTimer();
    }

    private void scan()
    {

    }

    private void emergencyFinishIfNeeded()
    {
        if(opmodeTimer.getElapsedTimeSeconds() >= 29)
        {
//            robot.turret.updateTurretPositionXY();
//            robot.sorterHardware.prepareNewMovement(0);
            setCurrentStep(END);
        }
    }
}
