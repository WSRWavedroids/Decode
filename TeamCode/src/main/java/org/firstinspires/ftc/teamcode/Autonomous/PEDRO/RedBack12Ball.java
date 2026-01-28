package org.firstinspires.ftc.teamcode.Autonomous.PEDRO;

import static org.firstinspires.ftc.teamcode.Core.ArtifactLocator.SlotState.*;
import static org.firstinspires.ftc.teamcode.Core.Robot.allianceSides.RED;
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

@Autonomous(name = "Red Back 12 Ball", group = "1. FABIO WITH PEDRO")
@Configurable // Panels
public class RedBack12Ball extends BlueBack12Ball {


    @Override
    public void init() {
        startPose = new Pose(144-50.188, 9.200, Math.toRadians(90));

        super.init();

        follower.setStartingPose(new Pose(144-50.188, 9.200, Math.toRadians(90)));

//        robot.alliance = RED;

        paths = new PathsForBack12Red(follower); // Build paths

        blackboard.put(ALLIANCE_KEY, "RED");

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    public static class PathsForBack12Red extends PathsForBack12Blue {
        public PathsForBack12Red(Follower follower) {
            super(follower);
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

            MoveToFireThirdPattern = follower
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
}
