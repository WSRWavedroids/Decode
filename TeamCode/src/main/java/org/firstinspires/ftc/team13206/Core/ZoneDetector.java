package org.firstinspires.ftc.team13206.Core;

public class ZoneDetector {
    private Robot robot;

    private static final double CENTER_TO_CORNER_LENGTH = Math.sqrt(40.5);

    public Vector2[] cornerPositions = {
            new Vector2(),
            new Vector2(),
            new Vector2(),
            new Vector2()
    };

    public ZoneDetector(Robot robot) {
        this.robot = robot;
    }

    public boolean isInFireZone() {
        updatePositions();
        for (Vector2 point : cornerPositions) {
            if (point.y >= findCloseZoneLine(point.x) | point.y <= findFarZoneLine(point.x)) {
                return true;
            }
        }
        return false;
    }

    private void updatePositions() {
        double heading = Math.toRadians(robot.robotHeading + 45);
        for (int i = 0; i <= 3; i++) {
            cornerPositions[i].x = robot.robotPosition.x + (CENTER_TO_CORNER_LENGTH * Math.cos(heading));
            cornerPositions[i].y = robot.robotPosition.x + (CENTER_TO_CORNER_LENGTH * Math.sin(heading));
            heading += Math.PI / 2;
        }
    }

    private double findCloseZoneLine(double x) {
        if (x <= 72) {
            return -x + 144;
        }
        else {
            return x;
        }
    }

    private double findFarZoneLine(double x) {
        if (x <= 72) {
            return x - 48;
        }
        else {
            return -x + 96;
        }
    }

}
