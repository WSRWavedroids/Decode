package org.firstinspires.ftc.teamcode.Core;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.Core.ArtifactLocator.SlotState.*;
import static org.firstinspires.ftc.teamcode.Core.BetaLauncherHardware.LauncherMode.IF_SAFE_NOW;
import static org.firstinspires.ftc.teamcode.Core.BetaLauncherHardware.LauncherMode.WAIT_FOREVER;
import static org.firstinspires.ftc.teamcode.Core.BetaLauncherHardware.LauncherMode.WAIT_FOR_TIME;
import static org.firstinspires.ftc.teamcode.Core.BetaLauncherHardware.LauncherSteps.*;
import static org.firstinspires.ftc.teamcode.Core.Robot.OpenClosed.*;
import static org.firstinspires.ftc.teamcode.Core.BetaSorterHardware.positionState.*;
import static org.firstinspires.ftc.teamcode.Core.ezPID.movementType.SPEED;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class BetaLauncherHardware {

    private Robot robot;
    public DcMotorEx motor;
    private static ezPID launcherPID;


    public BetaLauncherHardware(Robot robotFile) {
        robot = robotFile;
        motor = robot.launcherMotor1;
        // motor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
        motor.setDirection(REVERSE);
        waitingToFire = false;
    }

    public static double launcherCooldownDuration = 0.3;

    public boolean waitingToFire = false;
    boolean lockControls = false;
    boolean stopMotorAfter;
    boolean doneFiring = false;

    public static final int ticksPerRevolution = 28;
    public static final int revolutionsPerSecond = 100;
    public static final double toleranceRange = 350;

    public double velocityTarget;
    public double percentSpeed;

    public boolean onCooldown = false;

    public boolean wantToOpenDoor;

    LauncherMode mode;
    private double waitTime;
    private ElapsedTime waitForSafeTimer = new ElapsedTime();

    enum LauncherSteps {
        READY_FOR_COMMANDS,
        STALLING_UNTIL_SAFE, CHECK_IF_SAFE, WAIT_FOR_TIME_FOR_SAFE,
        REV_MOTOR, STALL_WHILE_MOTOR_REVVING, OPEN_DOOR, LAUNCHING, CLOSE_DOOR, RESET
    }
    public enum LauncherMode {
        WAIT_FOREVER, WAIT_FOR_TIME, IF_SAFE_NOW
    }
    LauncherSteps currentLauncherStep = READY_FOR_COMMANDS;
    private void nextStep(LauncherSteps nextStep) {
        currentLauncherStep = nextStep;
    }
    private ElapsedTime cooldownTimer = new ElapsedTime();

    public void updateLauncherHardware() {
        robot.telemetry.addLine("Untested launcher hardware, I choose you!");
        robot.telemetry.addData("Launcher step", currentLauncherStep);
        switch (currentLauncherStep) {
            case READY_FOR_COMMANDS:
                if (waitingToFire) {
                    waitingToFire = false;
                    doneFiring = false;
                    switch (mode) {
                        case IF_SAFE_NOW:
                            nextStep(CHECK_IF_SAFE);
                            break;
                        case WAIT_FOREVER:
                            nextStep(STALLING_UNTIL_SAFE);
                            break;
                        case WAIT_FOR_TIME:
                            nextStep(WAIT_FOR_TIME_FOR_SAFE);
                            break;
                    }
                }
                break;
            case STALLING_UNTIL_SAFE:
                if (robot.sorterHardware.fireSafeCheck()) {
                    nextStep(REV_MOTOR);
                }
                break;
            case WAIT_FOR_TIME_FOR_SAFE:
                if (robot.sorterHardware.fireSafeCheck()) {
                    // All good
                    nextStep(REV_MOTOR);
                } else if (waitForSafeTimer.seconds() >= waitTime) {
                    // Command timed out
                    nextStep(READY_FOR_COMMANDS);
                }
                break;
            case CHECK_IF_SAFE:
                if (robot.sorterHardware.fireSafeCheck()) {
                    nextStep(REV_MOTOR);
                } else {
                    nextStep(READY_FOR_COMMANDS);
                }
                break;
            case REV_MOTOR:
                setLauncherSpeed(percentSpeed);
                cooldownTimer.reset();
                nextStep(STALL_WHILE_MOTOR_REVVING);
                break;
            case STALL_WHILE_MOTOR_REVVING:
                if (motorSpeedCheck(velocityTarget) /*|| cooldownTimer.seconds() >= .75*/) {
                    nextStep(OPEN_DOOR);
                }
                break;
            case OPEN_DOOR:
                lockControls = true;
                onCooldown = true;
                wantToOpenDoor = true;
                robot.sorterHardware.moveDoor(OPEN);
                if (robot.sorterHardware.doorIs(OPEN)) {
                    cooldownTimer.reset();
                    nextStep(LAUNCHING);
                }
                break;
            case LAUNCHING:
                if (cooldownTimer.seconds() >= launcherCooldownDuration) {
                    nextStep(CLOSE_DOOR);
                }
                break;
            case CLOSE_DOOR:
                wantToOpenDoor = false;
                robot.sorterHardware.moveDoor(CLOSED);
                nextStep(RESET);

                break;
            case RESET:
                if (stopMotorAfter) setLauncherSpeed(0);
                robot.sorterLogic.findCurrentSlotInPosition(FIRE).setOccupied(EMPTY);
                lockControls = false;
                onCooldown = false;
                doneFiring = true;
                nextStep(READY_FOR_COMMANDS);
                break;
        }
    }

    public boolean doneFiring() {
        if (doneFiring) {
            doneFiring = false;
            return true;
        }
        else return false;
    }

    public void fireNowIfSafe(double speedTarget, boolean useSpeedTarget, boolean stopMotorAfter){
        readyFire(speedTarget, useSpeedTarget, stopMotorAfter);
        mode = IF_SAFE_NOW;
    }
    public void fireWithinTimeIfSafe(double speedTarget, boolean useSpeedTarget, boolean stopMotorAfter, double waitTime){
        readyFire(speedTarget, useSpeedTarget, stopMotorAfter);
        this.waitTime = waitTime;
        mode = WAIT_FOR_TIME;
        waitForSafeTimer.reset();
    }

    public void readyFire(double speedTarget, boolean useSpeedTarget) {
        if (lockControls) return;

        if (useSpeedTarget) percentSpeed = speedTarget;
        else percentSpeed = 1;

        waitingToFire = true;
        mode = WAIT_FOREVER;
    }

    public void readyFire() {
        this.readyFire(0, false, true);
    }
    public void readyFire(double speedTarget, boolean useSpeedTarget, boolean stopMotorAfter) {
        this.stopMotorAfter = stopMotorAfter;
        readyFire(speedTarget, useSpeedTarget);
    }

    public void setLauncherSpeed(double targetspeed) {
        velocityTarget = ticksPerRevolution * (revolutionsPerSecond * targetspeed);
        motor.setVelocity(velocityTarget);
    }

    public boolean motorSpeedCheck(double speedTarget) {
        if (atMaxSpeed()) {
            return true;
        }
        return (motor.getVelocity() > (-speedTarget - toleranceRange)) && (motor.getVelocity() < (-speedTarget + toleranceRange));
    }
    public boolean motorSpeedCheck() {
        return motorSpeedCheck(velocityTarget);
    }

    public boolean atMaxSpeed() {
        double voltageMax = -138.3958 * robot.voltageSensor.getVoltage() - 836.5097;
        return motor.getVelocity() > (voltageMax - toleranceRange) &&
                motor.getVelocity() < (voltageMax + toleranceRange);
    }
}
