package org.firstinspires.ftc.teamcode.Vision;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class WaveTag {
    //WaveTag is a custom class for moving the selected tag's 3d info between scripts

    public WaveTag() {
        // Haha so much code lol
    }
    public int tagID;

    public boolean currentlyDetected;

    public double distanceZ;
    public double distanceY;
    public double distanceX;

    public double angleX;
    public double angleY;

    public Pose3D tagPosFromRobot;
    public Pose3D robotFromTagPos;


}