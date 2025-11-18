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

import static android.os.SystemClock.sleep;

import com.bylazar.panels.Panels;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.ArtifactLocator;
import org.firstinspires.ftc.teamcode.Core.Robot;

/**
 * This is the autonomous mode. It moves the robot without us having to touch the controller.
 * Previous programmers really sucked at explaining what any of this meant, so we're trying to do better.
 * This is our third year now of using this file. It's kind of poetic and also adorable.
 */

public class AutonomousPlusPLUS {

    // This section tells the program all of the different pieces of hardware that are on our robot that we will use in the program.
    private ElapsedTime runtime = new ElapsedTime();

    public ElapsedTime stupidTimer = new ElapsedTime();
    public double speed = 0.6;
    public int sleepTime;
    public boolean inMarker;
    public double power;
    public double slidePos;

    //static TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    public Robot robot = null;

    public AutonomousPlusPLUS(Robot robot) {
        this.robot = robot;
        this.robot.panels = Panels.INSTANCE;
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
            while (robot.isWheelsBusy()) {
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

    public boolean checkMovement() {
        if (robot.isWheelsBusy()) {
            return false;
        } else {
            robot.stopAllMotors();
            robot.encoderRunningMode();
            //robot.encoderReset();
            return true;
        }
    }

    public void moveRobotForward(int ticks) {

        robot.setTargets("Forward", ticks); // Inverted... Lol
        robot.positionRunningMode();

        robot.powerSet(speed);
    }

    public void moveRobotForward(int ticks, long pause) {
        moveRobotForward(ticks);
        while (robot.isWheelsBusy()) {
            robot.tellMotorOutput();
            robot.panelsTelemetry.addData("FRD Position", robot.frontRightDrive.getCurrentPosition());
            robot.updateAllDaThings();
        }

        robot.stopAllMotors();
        robot.encoderRunningMode();
        sleep(pause);

        //robot.encoderReset();
    }

    public void moveRobotBackward(int ticks) {
        robot.setTargets("Backward", ticks);
        robot.positionRunningMode();
        robot.powerSet(speed);
    }

    public void moveRobotBackward(int ticks, long pause) {
        moveRobotForward(ticks);

        while (robot.isWheelsBusy()) {
            robot.tellMotorOutput();
        }

        robot.stopAllMotors();
        robot.encoderRunningMode();
        sleep(pause);
    }

    public void moveRobotLeft(int ticks) {
        robot.setTargets("Left", ticks);
        robot.positionRunningMode();
        robot.powerSet(speed);
    }

    public void moveRobotLeft(int ticks, long pause) {
        moveRobotLeft(ticks);

        while (robot.isWheelsBusy()) {
            robot.tellMotorOutput();
        }

        robot.stopAllMotors();
        robot.encoderRunningMode();
        sleep(pause);
    }

    public void moveRobotRight(int ticks) {
        robot.setTargets("Right", ticks);
        robot.positionRunningMode();
        robot.powerSet(speed);
    }

    public void moveRobotRight(int ticks, long pause) {
        moveRobotRight(ticks);

        while (robot.isWheelsBusy()) {
            robot.tellMotorOutput();
        }

        robot.stopAllMotors();
        robot.encoderRunningMode();
        sleep(pause);
    }

    public void turnRobotRight(int ticks) {
        robot.setTargets("Turn Right", ticks);
        robot.positionRunningMode();
        robot.powerSet(speed);
    }

    public void turnRobotRight(int ticks, long pause) {
        turnRobotRight(ticks);

        while (robot.isWheelsBusy()) {
            robot.tellMotorOutput();
        }

        robot.stopAllMotors();
        robot.encoderRunningMode();
        sleep(pause);
    }

    public void turnRobotLeft(int ticks) {
        robot.setTargets("Turn Left", ticks);
        robot.positionRunningMode();
        robot.powerSet(speed);
    }

    public void turnRobotLeft(int ticks, long pause) {
        turnRobotLeft(ticks);

        while (robot.isWheelsBusy()) {
            robot.tellMotorOutput();
        }

        robot.stopAllMotors();
        robot.encoderRunningMode();
        sleep(pause);
    }

    public void moveDiagonalRight(int ticks, long pause) {
        //This moves along the 45/225 axis, Positive ticks move forward and negative move back
        robot.setTargets("Diagonal Right", ticks);
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.powerSet(speed);

        while (robot.isWheelsBusy()) {
            robot.tellMotorOutput();
            robot.panelsTelemetry.addData("FRD Position", robot.frontRightDrive.getCurrentPosition());
            robot.panelsTelemetry.addData("FRD Position", robot.frontRightDrive.getVelocity());
            robot.updateAllDaThings();
        }

        robot.stopAllMotors();
        robot.encoderRunningMode();
        sleep(pause);
        //robot.encoderReset();

    }

    public void moveDiagonalLeft(int ticks, long pause) {
        //moves along the 135/315 axis, positive ticks move forward and negative ticks move back
        robot.setTargets("Diagonal Left", ticks);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.powerSet(speed);

        while (robot.isWheelsBusy()) {
            robot.tellMotorOutput();
            robot.panelsTelemetry.addData("FRD Position", robot.frontRightDrive.getCurrentPosition());
            robot.panelsTelemetry.addData("FRD Position", robot.frontRightDrive.getVelocity());
            robot.updateAllDaThings();
        }

        robot.stopAllMotors();
        robot.encoderRunningMode();
        sleep(pause);
        //robot.encoderReset();
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
        while (timer.milliseconds() < time)
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
        while (timer.milliseconds() < time)
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

    public void timeDriveLeft(long time, long pause) {//time is in milliseconds
        ElapsedTime timer = new ElapsedTime();
        robot.encoderRunningMode();
        timer.reset();
        while (timer.milliseconds() < time)
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
        while (timer.milliseconds() < time)
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
        while (timer.milliseconds() < time)
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
        while (timer.milliseconds() < time)
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
        while (timer.milliseconds() < time)
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






    public void calibrateDriveTrain(int tollerance, double pValue) {
        robot.frontLeftDrive.setTargetPositionTolerance(tollerance);
        robot.frontRightDrive.setTargetPositionTolerance(tollerance);
        robot.backLeftDrive.setTargetPositionTolerance(tollerance);
        robot.backRightDrive.setTargetPositionTolerance(tollerance);

        robot.frontLeftDrive.setPositionPIDFCoefficients(pValue);
        robot.frontRightDrive.setPositionPIDFCoefficients(pValue);
        robot.backLeftDrive.setPositionPIDFCoefficients(pValue);
        robot.backRightDrive.setPositionPIDFCoefficients(pValue);

    }

    public void prepareNextAction(long pause) {
        sleep(pause);
        //robot.encoderReset();
    }



    public int convertInchesToTicks(int inches){
        int ticks = (int) ((537.6 * inches) / (3.77953 * 3.1415926535));
        return ticks;
    }

    public void stallTillFalse(boolean condition) {
        while (condition)
        {
            robot.updateAllDaThings();
            robot.telemetry.update();
        }
    }

    public void stallTillTrue(boolean condition)
    {
        while(!condition)
        {
            robot.updateAllDaThings();
            robot.sorterHardware.moveDoor();
            robot.sorterHardware.updateSorterHardware();
            robot.sorterHardware.runPIDMotorStuffLol();
            robot.launcher.updateLauncherHardware();
            robot.launcher.timerCheck();

            if(condition)
            {
                break;
            }
        }
    }

    void stallForSpin(boolean condition, int ticks)
    {
        int shortTermRef = robot.sorterHardware.findFastestRotationInTicks(robot.sorterHardware.motor.getCurrentPosition(), ticks);
        robot.sorterHardware.reference = shortTermRef;
        while(!condition)
        {

            robot.sorterHardware.reference  = robot.sorterHardware.findFastestRotationInTicks(robot.sorterHardware.motor.getCurrentPosition(), ticks);
            robot.updateAllDaThings();

            robot.sorterHardware.reference = shortTermRef;
            robot.sorterHardware.updateSorterHardware();
            robot.sorterHardware.runPIDMotorStuffLol();
            robot.launcher.updateLauncherHardware();
            robot.launcher.timerCheck();
            robot.launcher.runHammer();
            robot.telemetry.addData("Spinning...", "Or stuck in loop :(");
            robot.telemetry.addData("we in?", robot.sorterHardware.positionedCheck());
            if(robot.sorterHardware.positionedCheck())
            {
                sleep(100);
                if(robot.sorterHardware.positionedCheck())
                {
                    break;
                }
            }
            robot.telemetry.update();
        }
    }

    void stallForTime(double time)
    {
        stupidTimer.reset();
        while(stupidTimer.seconds() < time)
        {
            robot.updateAllDaThings();
            robot.sorterHardware.moveDoor();
            robot.sorterHardware.updateSorterHardware();
            robot.sorterHardware.runPIDMotorStuffLol();

            robot.launcher.updateLauncherHardware();
            robot.launcher.timerCheck();
            //robot.launcher.runHammer();
            {

            }
        }
    }

    void stallForCondition(boolean condition)
    {
        while(!condition)
        {
            robot.updateAllDaThings();
            robot.sorterHardware.moveDoor();
            robot.sorterHardware.updateSorterHardware();
            robot.sorterHardware.runPIDMotorStuffLol();
            robot.launcher.updateLauncherHardware();
            robot.launcher.timerCheck();
            robot.launcher.runHammer();

            if(condition)
            {
                break;
            }
        }
    }


    public void fireInSequence(ArtifactLocator.slot one, ArtifactLocator.slot two, ArtifactLocator.slot three)
    {
        int i = 0;
        boolean stalling = false;
        if (i <= 11) {
            if (!stalling) {
                robot.launcher.setLauncherSpeed(1);
                robot.sorterHardware.prepareNewMovement(
                        robot.sorterHardware.motor.getCurrentPosition(),
                        one.getFirePosition());
            }

        }
        stallForSpin(robot.sorterHardware.positionedCheck(), one.getFirePosition());
        stallForSpin(robot.sorterHardware.positionedCheck(), one);
        stallForSpin(robot.sorterHardware.positionedCheck(), one);
        stallForSpin(robot.sorterHardware.positionedCheck(), one);
        stallForSpin(robot.sorterHardware.positionedCheck(), one);
        stallForSpin(robot.sorterHardware.positionedCheck(), one);
        stallForSpin(robot.sorterHardware.positionedCheck(), one);
        stallForSpin(robot.sorterHardware.positionedCheck(), one);
        stallForSpin(robot.sorterHardware.positionedCheck(), one);
        stallForSpin(robot.sorterHardware.positionedCheck(), one);
        robot.doorServo.setPosition(robot.sorterHardware.doorOpenPosition);
        sleep(500);
        robot.doorServo.setPosition(robot.sorterHardware.doorClosedPosition);
        stallForTime(0.5);


        robot.launcher.setLauncherSpeed(1);
        stallForSpin(robot.sorterHardware.positionedCheck(), two);
        stallForSpin(robot.sorterHardware.positionedCheck(), two);
        stallForSpin(robot.sorterHardware.positionedCheck(), two);
        stallForSpin(robot.sorterHardware.positionedCheck(), two);
        stallForSpin(robot.sorterHardware.positionedCheck(), two);
        stallForSpin(robot.sorterHardware.positionedCheck(), two);
        stallForSpin(robot.sorterHardware.positionedCheck(), two);
        stallForSpin(robot.sorterHardware.positionedCheck(), two);
        stallForSpin(robot.sorterHardware.positionedCheck(), two);
        stallForSpin(robot.sorterHardware.positionedCheck(), two);
        robot.doorServo.setPosition(robot.sorterHardware.doorOpenPosition);
        sleep(500);
        robot.doorServo.setPosition(robot.sorterHardware.doorClosedPosition);
        stallForTime(0.5);


        robot.launcher.setLauncherSpeed(1);
        stallForSpin(robot.sorterHardware.positionedCheck(), three);
        stallForSpin(robot.sorterHardware.positionedCheck(), three);
        stallForSpin(robot.sorterHardware.positionedCheck(), three);
        stallForSpin(robot.sorterHardware.positionedCheck(), three);
        stallForSpin(robot.sorterHardware.positionedCheck(), three);
        stallForSpin(robot.sorterHardware.positionedCheck(), three);
        stallForSpin(robot.sorterHardware.positionedCheck(), three);
        stallForSpin(robot.sorterHardware.positionedCheck(), three);
        stallForSpin(robot.sorterHardware.positionedCheck(), three);
        stallForSpin(robot.sorterHardware.positionedCheck(), three);
        robot.doorServo.setPosition(robot.sorterHardware.doorOpenPosition);
        sleep(500);
        robot.doorServo.setPosition(robot.sorterHardware.doorClosedPosition);
        stallForTime(0.5);

        //reset to safe
        robot.launcher.setLauncherSpeed(0);
        stallForSpin(robot.sorterHardware.positionedCheck(), robot.sorterHardware.positions[0]);
        stallForSpin(robot.sorterHardware.positionedCheck(), robot.sorterHardware.positions[0]);
    }







    public void safeSorterSpin(int targetPosition)
    {
        while(robot.sorterHardware.motor.getCurrentPosition() > targetPosition - robot.sorterHardware.tickTolerance  && robot.sorterHardware.motor.getCurrentPosition() < targetPosition + robot.sorterHardware.tickTolerance)
        {
            //
        }
    }






    /**
     * This is the autonomous mode. It moves the robot without us having to touch the controller.
     * Previous programmers really sucked at explaining what any of this meant, so we're trying to do better.
     * This is our third year now of using this file. It's kind of poetic and also adorable.
     */

}
