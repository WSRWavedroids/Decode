package org.firstinspires.ftc.teamcode.Autonomous;
/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Vision.Limelight_Randomization_Scanner;
//import org.firstinspires.ftc.teamcode.OLD.Autonomous.AprilTags.MayFlowers;

/**
 * This is the autonomous mode. It moves the robot without us having to touch the controller.
 * Previous programmers really sucked at explaining what any of this meant, so we're trying to do better.
 * This is our third year now of using this file. It's kind of poetic and also adorable.
 */

public class AutonomousPLUS extends LinearOpMode {

    // This section tells the program all of the different pieces of hardware that are on our robot that we will use in the program.
    private ElapsedTime runtime = new ElapsedTime();

    public Limelight_Randomization_Scanner randomization;

    public double speed = 0.6;
    public int sleepTime;
    public boolean inMarker;
    public double power;
    public double slidePos;

    //DO NOT DELETE THIS LINE! CAPITALIZATION IS VERY IMPORTANT!!!
    public Robot robot = null;

    /*public AutonomousPLUS(){
        robot = new Robot(hardwareMap, telemetry, this);
    }*/
    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry, this);
    }

    //These are the basic functions for mechnum movement during auto... Don't mess with these unless something is inverted
    // Remember Without ODO pods there will be some inconsistency due to mechnum slippage

    /**
     * Moves the robot in the provided X and Y directions and turns using... some value.
     *
     * Positive moveRight = Right
     * Positive moveForward = Forward
     * Positive turn = Clockwise
     * @param moveRight negative = left, positive = right
     * @param moveForward positive = forwards, negative = backwards
     * @param turn negative = counterclockwise, positive = clockwise
     * @param waitForCompletion Stall until movement complete
     * @param pauseMS Pause in milliseconds
     */
    public void moveXY(int moveRight, int moveForward, int turn, boolean waitForCompletion, long pauseMS) {

        int[] motorTicks = new int[4];

        motorTicks[0] = (-moveForward + moveRight + turn);
        motorTicks[1] = (-moveForward - moveRight - turn);
        motorTicks[2] = (-moveForward - moveRight + turn);
        motorTicks[3] = (-moveForward + moveRight - turn);

        robot.frontLeftDrive.setTargetPosition(-motorTicks[0] + robot.frontLeftDrive.getCurrentPosition());
        robot.frontRightDrive.setTargetPosition(-motorTicks[1] + robot.frontRightDrive.getCurrentPosition());
        robot.backLeftDrive.setTargetPosition(-motorTicks[2] + robot.backLeftDrive.getCurrentPosition());
        robot.backRightDrive.setTargetPosition(-motorTicks[3] + robot.backRightDrive.getCurrentPosition());

        if (waitForCompletion) {
            while (opModeIsActive() && robot.isWheelsBusy()) {
                robot.tellMotorOutput();
                robot.updateAllDaThings();
            } // And we stall...
        }

        sleep(pauseMS);
    }
    // No turn
    public void moveXY(int moveRight, int moveForward, boolean waitForCompletion, long pauseMS) {
        moveXY(moveRight,moveForward,0, waitForCompletion, pauseMS);
    }
    //No pauseMS
    public void moveXY(int moveRight, int moveForward, int turn, boolean waitForCompletion) {
        moveXY(moveRight,moveForward,turn,waitForCompletion,0);
    }
    // No turn or pauseMS
    public void moveXY(int moveRight, int moveForward, boolean waitForCompletion) {
        moveXY(moveRight,moveForward,0,waitForCompletion,0);
    }

    public void moveRobotForward(int ticks, long pause) {
        if (opModeIsActive()) {
            robot.setTargets("Forward", ticks); // Inverted... Lol
            robot.positionRunningMode();
        }
        robot.powerSet(speed);

        while (opModeIsActive() && robot.isWheelsBusy()) {
            robot.tellMotorOutput();
            robot.updateAllDaThings();
        }

        robot.stopAllMotors();
        robot.encoderRunningMode();
        sleep(pause);
        robot.encoderReset();
    }


    public void moveRobotBackward(int ticks, long pause) {
        if (opModeIsActive()) {
            robot.setTargets("Backward", ticks);
            robot.positionRunningMode();
            robot.powerSet(speed);

            while (opModeIsActive() && robot.isWheelsBusy()) {
                robot.tellMotorOutput();
                robot.updateAllDaThings();
            }

            robot.stopAllMotors();
            robot.encoderRunningMode();
            sleep(pause);
            robot.encoderReset();
        }
    }

    public void moveRobotLeft(int ticks, long pause) {

        if (opModeIsActive()) {
            robot.setTargets("Left", ticks);
            robot.positionRunningMode();
            robot.powerSet(speed);

            while (opModeIsActive() && robot.isWheelsBusy()) {
                robot.tellMotorOutput();
                robot.updateAllDaThings();
            }

            robot.stopAllMotors();
            robot.encoderRunningMode();
            sleep(pause);
            robot.encoderReset();
        }
    }

    public void moveRobotRight(int ticks, long pause) {

        if (opModeIsActive()) {
            robot.setTargets("Right", ticks);
            robot.positionRunningMode();
            robot.powerSet(speed);

            while (opModeIsActive() && robot.isWheelsBusy()) {
                robot.tellMotorOutput();
                robot.updateAllDaThings();
            }

            robot.stopAllMotors();
            robot.encoderRunningMode();
            sleep(pause);
            robot.encoderReset();
        }
    }

    public void turnRobotRight(int ticks, long pause) {

        if (opModeIsActive()) {
            robot.setTargets("Turn Right", ticks);
            robot.positionRunningMode();
            robot.powerSet(speed);

            while (opModeIsActive() && robot.isWheelsBusy()) {
                robot.tellMotorOutput();
                robot.updateAllDaThings();
            }

            robot.stopAllMotors();
            robot.encoderRunningMode();
            sleep(pause);
            robot.encoderReset();
        }
    }

    public void turnRobotLeft(int ticks, long pause) {

        if (opModeIsActive()) {
            robot.setTargets("Turn Left", ticks);
            robot.positionRunningMode();
            robot.powerSet(speed);

            while (opModeIsActive() && robot.isWheelsBusy()) {
                robot.tellMotorOutput();
                robot.updateAllDaThings();
            }

            robot.stopAllMotors();
            robot.encoderRunningMode();
            sleep(pause);
            robot.encoderReset();

        }
    }

    public void moveDiagonalRight(int ticks, long pause) {
        //This moves along the 45/225 axis, Positive ticks move forward and negative move back
        if (opModeIsActive()) {
            robot.setTargets("Diagonal Right", ticks);
            robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.powerSet(speed);

            while (opModeIsActive() && robot.isWheelsBusy()) {
                robot.tellMotorOutput();
                robot.updateAllDaThings();
            }

            robot.stopAllMotors();
            robot.encoderRunningMode();
            sleep(pause);
            robot.encoderReset();
        }
    }

    public void moveDiagonalLeft(int ticks, long pause) {
        //moves along the 135/315 axis, positive ticks move forward and negative ticks move back
        if (opModeIsActive()) {
            robot.setTargets("Diagonal Left", ticks);
            robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.powerSet(speed);

            while (opModeIsActive() && robot.isWheelsBusy()) {
                robot.tellMotorOutput();
                robot.updateAllDaThings();
            }

            robot.stopAllMotors();
            robot.encoderRunningMode();
            sleep(pause);
            robot.encoderReset();
        }
    }

    public void timeDriveForward(long time, long pause)
    {//time is in milliseconds
        ElapsedTime timer = new ElapsedTime();
        robot.encoderRunningMode();
        timer.reset();
        while (timer.milliseconds() < time)
        {
            robot.frontLeftDrive.setPower(-speed);
            robot.backLeftDrive.setPower(-speed);
            robot.frontRightDrive.setPower(-speed);
            robot.backRightDrive.setPower(-speed);

            robot.updateAllDaThings();
        }
        robot.stopAllMotors();
        sleep(pause);
    }

    public void timeDriveBackward(long time, long pause)
    {//time is in milliseconds
        ElapsedTime timer = new ElapsedTime();
        robot.encoderRunningMode();
        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < time)
        {
            robot.frontLeftDrive.setPower(speed);
            robot.backLeftDrive.setPower(speed);
            robot.frontRightDrive.setPower(speed);
            robot.backRightDrive.setPower(speed);

            robot.updateAllDaThings();
        }
        robot.stopAllMotors();
        sleep(pause);
    }

    public void timeDriveRight(long time, long pause)
    {//time is in milliseconds
        ElapsedTime timer = new ElapsedTime();
        robot.encoderRunningMode();
        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < time)
        {
            robot.frontLeftDrive.setPower(-speed);
            robot.backLeftDrive.setPower(speed);
            robot.frontRightDrive.setPower(speed);
            robot.backRightDrive.setPower(-speed);

            robot.updateAllDaThings();
        }
        robot.stopAllMotors();
        sleep(pause);
    }

    public void timeDriveLeft(long time, long pause)
    {//time is in milliseconds
        ElapsedTime timer = new ElapsedTime();
        robot.encoderRunningMode();
        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < time)
        {
            robot.frontLeftDrive.setPower(speed);
            robot.backLeftDrive.setPower(-speed);
            robot.frontRightDrive.setPower(-speed);
            robot.backRightDrive.setPower(speed);

            robot.updateAllDaThings();
        }
        robot.stopAllMotors();
        sleep(pause);
    }

    public void timeTurnleft(long time, long pause)
    {//time is in milliseconds
        ElapsedTime timer = new ElapsedTime();
        robot.encoderRunningMode();
        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < time)
        {
            robot.frontLeftDrive.setPower(speed);
            robot.backLeftDrive.setPower(speed);
            robot.frontRightDrive.setPower(-speed);
            robot.backRightDrive.setPower(-speed);

            robot.updateAllDaThings();
        }
        robot.stopAllMotors();
        sleep(pause);
    }

    public void timeTurnRight(long time, long pause)
    {//time is in milliseconds
        ElapsedTime timer = new ElapsedTime();
        robot.encoderRunningMode();
        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < time)
        {
            robot.frontLeftDrive.setPower(-speed);
            robot.backLeftDrive.setPower(-speed);
            robot.frontRightDrive.setPower(speed);
            robot.backRightDrive.setPower(speed);

            robot.updateAllDaThings();
        }
        robot.stopAllMotors();
        sleep(pause);
    }

    public void timeDiagonalRight(long time, long pause, int PosOneForward_MinusOneBack)
    {// This moves along the 45/225 axis. Changing the last int to -1 will make it go back, pos 1 will go forward
        ElapsedTime timer = new ElapsedTime();
        robot.encoderRunningMode();
        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < time)
        {
            robot.frontLeftDrive.setPower(-speed *  PosOneForward_MinusOneBack);
            robot.backLeftDrive.setPower(0);
            robot.frontRightDrive.setPower(0);
            robot.backRightDrive.setPower(-speed * PosOneForward_MinusOneBack);

            robot.updateAllDaThings();
        }
        robot.stopAllMotors();
        sleep(pause);
    }

    public void timeDiagonalLeft(long time, long pause, int PosOneForward_MinusOneBack)
    {//Moves along the 135/315 degree axis. Changing the last int to -1 will make it go back, pos 1 will go forward
        ElapsedTime timer = new ElapsedTime();
        robot.encoderRunningMode();
        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < time)
        {
            robot.frontLeftDrive.setPower(0);
            robot.backLeftDrive.setPower(-speed * PosOneForward_MinusOneBack);
            robot.frontRightDrive.setPower(-speed * PosOneForward_MinusOneBack);
            robot.backRightDrive.setPower(0);

            robot.updateAllDaThings();
        }
        robot.stopAllMotors();
        sleep(pause);
    }



    public void prepareAuto() {


        robot.encoderReset();



    }




    public void setDrivetrainTolerances(int ticks) {
        robot.frontLeftDrive.setTargetPositionTolerance(ticks);
        robot.frontRightDrive.setTargetPositionTolerance(ticks);
        robot.backLeftDrive.setTargetPositionTolerance(ticks);
        robot.backRightDrive.setTargetPositionTolerance(ticks);
    }

    public void prepareNextAction(long pause) {
        sleep(pause);
        robot.encoderReset();
    }



    public int convertInchesToTicks(int inches){
        int ticks = (int) ((537.6 * inches) / (3.77953 * 3.1415926535));
        return ticks;
    }

    public void stallTillFalse(boolean condition) {
        while (condition)
        {
            robot.updateAllDaThings();
            sleep(1);
        }
    }

    public void stallTillTrue(boolean condition)
    {
        while (!condition)
        {
            robot.updateAllDaThings();
            sleep(1);
        }
    }




    /**
     * This is the autonomous mode. It moves the robot without us having to touch the controller.
     * Previous programmers really sucked at explaining what any of this meant, so we're trying to do better.
     * This is our third year now of using this file. It's kind of poetic and also adorable.
     */

}
