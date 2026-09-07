package org.firstinspires.ftc.team13206.Core;

import static org.firstinspires.ftc.team13206.Core.ArtifactLocator.SlotState.*;
import static org.firstinspires.ftc.team13206.Core.LauncherHardware.LauncherMode.IF_SAFE_NOW;
import static org.firstinspires.ftc.team13206.Core.LauncherHardware.LauncherMode.WAIT_FOREVER;
import static org.firstinspires.ftc.team13206.Core.LauncherHardware.LauncherMode.WAIT_FOR_TIME;
import static org.firstinspires.ftc.team13206.Core.LauncherHardware.LauncherSteps.*;
import static org.firstinspires.ftc.team13206.Core.Robot.allianceSides.BLUE;
import static org.firstinspires.ftc.team13206.Core.SorterHardware.PositionState.*;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
public class LauncherHardware {

    private Robot robot;
    public DcMotorEx motor1, motor2;
    private TurretLogic turret;
    private PIDMotorGroup launcherMotors;

    public ezPID launcherPID;




    public LauncherHardware(Robot robotFile) {
        robot = robotFile;
        motor1 = robot.launcherMotorOne;
        motor2 = robot.launcherMotorTwo;
        launcherMotors = new PIDMotorGroup(motor1, motor2);
        launcherMotors.setDirections(-1, -1);
        launcherPID = new ezPID(motor1, 28, p, i, d, f, 1, toleranceRange, ezPID.movementType.SPEED);
        turret = robot.turret;
        // motor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
        waitingToFire = false;
        activeFiringSlot = robot.sorterLogic.noSlot;
    }

    public static double launcherCooldownDuration = 0.3;
    public static double flickTime = 0.1;

    public boolean waitingToFire = false;
    boolean lockControls = false;
    boolean stopMotorAfter;
    boolean firing = false;
    boolean activeFiring = false;
    public ArtifactLocator.Slot activeFiringSlot;

    public static final int ticksPerRevolution = 28;
    public static final int revolutionsPerSecond = 100;
    public static double toleranceRange = 40;

    public static double velocityTarget;
    public double percentSpeed;
    public int steadiness = 0;
    public final int steadinessThreshold = 1;

    public boolean onCooldown = false;
    public static boolean manualTuneMode = false;

    LauncherMode mode;
    private double waitTime;
    private ElapsedTime waitForSafeTimer = new ElapsedTime();

    public static double p = 0.0045;
    public static double i = 0;
    public static double d = 0;
    public static double f = 0.0004;

    public enum LauncherSteps {
        READY_FOR_COMMANDS,
        STALLING_UNTIL_SAFE, CHECK_IF_SAFE, WAIT_FOR_TIME_FOR_SAFE,
        REV_MOTOR, STALL_WHILE_MOTOR_REVVING, FLICK, UNFLICK, LAUNCHING
    }
    public enum LauncherMode {
        WAIT_FOREVER, WAIT_FOR_TIME, IF_SAFE_NOW
    }
    LauncherSteps currentLauncherStep = READY_FOR_COMMANDS;

    public LauncherSteps getCurrentLauncherStep() {
        return currentLauncherStep;
    }

    private ElapsedTime cooldownTimer = new ElapsedTime();

    public void updateLauncherHardware() {
        robot.panelsTelemetry.addData("Launcher Motor 1 Velocity", motor1.getVelocity());
        robot.panelsTelemetry.addData("Launcher Motor 2 Velocity", motor2.getVelocity());
        robot.panelsTelemetry.addData("Target Launcher Velocity", velocityTarget);
        robot.panelsTelemetry.addData("LL Distance", robot.targetTag.distanceZ);

        switch (currentLauncherStep) {
            case READY_FOR_COMMANDS:
                firing = false;
                if (waitingToFire) {
                    waitingToFire = false;
                    firing = true;
                    switch (mode) {
                        case IF_SAFE_NOW:
                            currentLauncherStep = CHECK_IF_SAFE;
                            break;
                        case WAIT_FOREVER:
                            currentLauncherStep = STALLING_UNTIL_SAFE;
                            break;
                        case WAIT_FOR_TIME:
                            currentLauncherStep = WAIT_FOR_TIME_FOR_SAFE;
                            break;
                    }
                }
                break;
            case STALLING_UNTIL_SAFE:
                if (robot.sorterHardware.fireSafeCheck()) {
                    activeFiringSlot = robot.sorterLogic.findCurrentSlotInPosition(FIRE);
                    currentLauncherStep = REV_MOTOR;
                }
                break;
            case WAIT_FOR_TIME_FOR_SAFE:
                if (robot.sorterHardware.fireSafeCheck()) {
                    activeFiringSlot = robot.sorterLogic.findCurrentSlotInPosition(FIRE);
                    // All good
                    currentLauncherStep = REV_MOTOR;
                } else if (waitForSafeTimer.seconds() >= waitTime) {
                    // Command timed out
                    currentLauncherStep = READY_FOR_COMMANDS;
                }
                break;
            case CHECK_IF_SAFE:
                if (robot.sorterHardware.fireSafeCheck()) {
                    activeFiringSlot = robot.sorterLogic.findCurrentSlotInPosition(FIRE);
                    currentLauncherStep = REV_MOTOR;
                } else {
                    currentLauncherStep = READY_FOR_COMMANDS;
                }
                break;
            case REV_MOTOR:
                if (!activeFiringSlot.exists()) {
                    activeFiringSlot = robot.sorterLogic.findCurrentSlotInPosition(FIRE);
                }
                setPerfectLauncherVelocity();
                cooldownTimer.reset();
                currentLauncherStep = STALL_WHILE_MOTOR_REVVING;
                break;
            case STALL_WHILE_MOTOR_REVVING:
                if (!activeFiringSlot.exists()) {
                    activeFiringSlot = robot.sorterLogic.findCurrentSlotInPosition(FIRE);
                }
                setPerfectLauncherVelocity();
                if ((motorSpeedCheck(velocityTarget) && motorSteady() && robot.turret.positioned() || cooldownTimer.seconds() >= 5)) {
                    currentLauncherStep = FLICK;
                }
                break;
            case FLICK:
                if (!activeFiringSlot.exists()) {
                    activeFiringSlot = robot.sorterLogic.findCurrentSlotInPosition(FIRE);
                }
                lockControls = true;
                activeFiring = true;
                onCooldown = true;
                robot.sorterHardware.flick();
                cooldownTimer.reset();

                currentLauncherStep = UNFLICK;
                break;
            case UNFLICK:
                if (!activeFiringSlot.exists()) {
                    activeFiringSlot = robot.sorterLogic.findCurrentSlotInPosition(FIRE);
                }

                if(robot.sorterHardware.flickyInPosition() || cooldownTimer.seconds() >= flickTime) {
                    robot.sorterHardware.resetFlicky();
                    currentLauncherStep = LAUNCHING;
                }
                break;
            case LAUNCHING:
                if (!activeFiringSlot.exists()) {
                    activeFiringSlot = robot.sorterLogic.findCurrentSlotInPosition(FIRE);
                }

                if (stopMotorAfter && cooldownTimer.seconds() >= launcherCooldownDuration && cooldownTimer.seconds() >= flickTime * 1.5) {
                    reset();
                }
                else if (robot.sorterHardware.flickyInPosition() || cooldownTimer.seconds() >= flickTime * 1.5) {
                    reset();
                }
                break;
        }

        if (motorSpeedCheck()) {
            steadiness += 1;
        } else {
            steadiness = 0;
        }

        launcherPID.changeBehaviorValues(p, i, d, f, 1, toleranceRange);
        launcherPID.runCalledPID(velocityTarget);
        motor2.setPower(motor1.getPower());
    }

    private void reset() {
        // Stop the motor if requested
        if (stopMotorAfter) setLauncherVelocity(0);

        // Set the slot to empty now that we fired its contents
        activeFiringSlot.setOccupied(EMPTY);
        activeFiringSlot.release();

        activeFiringSlot = robot.sorterLogic.noSlot;

        // Update the booleans
        lockControls = false;
        activeFiring = false;
        onCooldown = false;
        firing = false;

        // All done, ready for the next one
        currentLauncherStep = READY_FOR_COMMANDS;
    }

    public boolean doneFiring() {
        return !firing;
    }
    public boolean isInFireSequence() {
        return firing;
    }
    public boolean isActivelyFiring() {
        return activeFiring;
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
        else percentSpeed = 0.5;

        waitingToFire = true;
        stopMotorAfter = false;
        mode = WAIT_FOREVER;
    }

    public void readyFire() {
        this.readyFire(0, false, false);
    }
    public void readyFire(double speedTarget, boolean useSpeedTarget, boolean stopMotorAfter) {
        readyFire(speedTarget, useSpeedTarget);
        this.stopMotorAfter = stopMotorAfter;
    }

    @Deprecated
    public void setLauncherSpeed(double targetSpeed) {
        velocityTarget = ticksPerRevolution * revolutionsPerSecond * targetSpeed;
    }

    public void setLauncherVelocity(double targetVelocity) {
        if (!manualTuneMode) {
            velocityTarget = targetVelocity;
        }
    }
    public void setPerfectLauncherVelocity() {
        setLauncherVelocity(findBestMotorVelocity(robot.targetTag.distanceZ));
    }

    public boolean motorSpeedCheck(double speedTarget) {
        /*if (atMaxSpeed()) {
            return true;
        }*/
        double realVelocity = motor1.getVelocity();
        return (realVelocity >= (speedTarget - toleranceRange)) && (realVelocity <= (speedTarget + toleranceRange));
    }
    public boolean motorSpeedCheck() {
        return motorSpeedCheck(velocityTarget);
    }

    //TODO retune this
    public boolean atMaxSpeed() {
        double voltageMax = -138.3958 * robot.voltageSensor.getVoltage() - 836.5097;
        return motor1.getVelocity() >= (voltageMax - toleranceRange);
    }

    public boolean motorSteady() {
        return steadiness >= steadinessThreshold;
    }

    public double findBestMotorVelocity(double input) {

        if(!robot.targetTag.currentlyDetected)
        {
            double targetX;
            double targetY;

            if(robot.alliance.equals(BLUE)) {
                targetX = 12;
                targetY = 132;
            }
            else
            {
                targetX = 132;
                targetY = 132;
            }

            input = Math.sqrt(Math.pow(robot.turretPosition.x - targetX, 2) + Math.pow(robot.turretPosition.y - targetY, 2));
            input /= 39.37; //convert to meters

        }
        return (21.20299 * Math.pow(input, 4)) +
                (-233.8409 * Math.pow(input, 3)) +
                (966.85113 * Math.pow(input, 2)) +
                (-1610.42433 * input) +
                2186.7502;
    }

    public boolean launcherOn() {
        return velocityTarget != 0;
    }
}
