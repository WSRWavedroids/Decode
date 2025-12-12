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
        launcherCooldownDuration = 0.25;
    }

    boolean lockControls = false;


    enum LauncherSteps {
        READY_FOR_COMMANDS, STALLING_UNTIL_SAFE, CHECK_IF_SAFE, REV_MOTOR, OPEN_DOOR, LAUNCHING, CLOSE_DOOR, RESET
    }
    LauncherSteps currentLauncherStep = READY_FOR_COMMANDS;
    private void nextStep(LauncherSteps nextStep) {
        currentLauncherStep = nextStep;
    }
    private ElapsedTime cooldownTimer = new ElapsedTime();
    @Override
    public void updateLauncherHardware() {
        robot.telemetry.addLine("Untested launcher hardware, I choose you!");
        switch (currentLauncherStep) {
            case READY_FOR_COMMANDS:
                if (waitingToFire) {
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
                if (motorSpeedCheck(velocityTarget)) {
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
                    onCooldown = false;
                    nextStep(CLOSE_DOOR);
                }
                break;
            case CLOSE_DOOR:
                wantToOpenDoor = false;
                robot.sorterHardware.moveDoor(CLOSED);
                if (!robot.sorterHardware.openCheck()){
                    nextStep(RESET);
                }
                break;
            case RESET:
                setLauncherSpeed(0);
                robot.sorterLogic.findCurrentSlotInPosition(FIRE).setOccupied(EMPTY);
                lockControls = false;
                nextStep(READY_FOR_COMMANDS);
                break;
        }
    }

    @Override
    public void readyFire(double speedTarget, boolean useSpeedTarget) {
        if (lockControls) return;

        if (useSpeedTarget) velocityTarget = speedTarget;
        else velocityTarget = 1;

        super.waitingToFire = true;
    }
}
