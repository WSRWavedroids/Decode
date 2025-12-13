package org.firstinspires.ftc.teamcode.Core;

import static org.firstinspires.ftc.teamcode.Core.ArtifactLocator.SlotState.EMPTY;
import static org.firstinspires.ftc.teamcode.Core.BetaLauncherHardware.LauncherSteps.*;
import static org.firstinspires.ftc.teamcode.Core.Robot.OpenClosed.*;
import static org.firstinspires.ftc.teamcode.Core.SorterHardware.positionState.FIRE;

import com.qualcomm.robotcore.util.ElapsedTime;

public class BetaLauncherHardware extends LauncherHardware {

    private Robot robot;
    public BetaLauncherHardware(Robot robotFile) {
        super(robotFile);
        this.robot = super.robot;
        launcherCooldownDuration = 0.5;
    }

    boolean lockControls = false;
    boolean stopMotorAfter;
    boolean doneFiring = false;


    enum LauncherSteps {
        READY_FOR_COMMANDS, STALLING_UNTIL_SAFE, CHECK_IF_SAFE, REV_MOTOR,
        STALL_WHILE_MOTOR_REVVING, OPEN_DOOR, LAUNCHING, CLOSE_DOOR, RESET
    }
    LauncherSteps currentLauncherStep = READY_FOR_COMMANDS;
    private void nextStep(LauncherSteps nextStep) {
        currentLauncherStep = nextStep;
    }
    private ElapsedTime cooldownTimer = new ElapsedTime();
    @Override
    public void updateLauncherHardware() {
        robot.telemetry.addLine("Untested launcher hardware, I choose you!");
        robot.telemetry.addData("Launcher step", currentLauncherStep);
        switch (currentLauncherStep) {
            case READY_FOR_COMMANDS:
                if (waitingToFire) {
                    waitingToFire = false;
                    doneFiring = false;
                    nextStep(STALLING_UNTIL_SAFE);
                }
                break;
            case STALLING_UNTIL_SAFE:
                if (robot.sorterHardware.fireSafeCheck()) {
                    nextStep(REV_MOTOR);
                }
                break;
            case CHECK_IF_SAFE:
                break;
            case REV_MOTOR:
                setLauncherSpeed(velocityTarget);
                cooldownTimer.reset();
                nextStep(STALL_WHILE_MOTOR_REVVING);
                break;
            case STALL_WHILE_MOTOR_REVVING:
                if (motorSpeedCheck(velocityTarget) || cooldownTimer.seconds() >= 1) {
                    nextStep(OPEN_DOOR);
                }
                break;
            case OPEN_DOOR:
                lockControls = true;
                onCooldown = true;
                wantToOpenDoor = true;
                robot.sorterHardware.moveDoor(OPEN);
                if (robot.sorterHardware.openCheck()) {
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

    @Override
    public void readyFire(double speedTarget, boolean useSpeedTarget) {
        if (lockControls) return;

        if (useSpeedTarget) velocityTarget = speedTarget;
        else velocityTarget = 1;

        super.waitingToFire = true;
    }
    @Override
    public void readyFire() {
        this.readyFire(0, false, true);
    }
    public void readyFire(double speedTarget, boolean useSpeedTarget, boolean stopMotorAfter) {
        this.stopMotorAfter = stopMotorAfter;
        readyFire(speedTarget, useSpeedTarget);
    }
}
