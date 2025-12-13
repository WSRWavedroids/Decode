package org.firstinspires.ftc.teamcode.Core;

import static org.firstinspires.ftc.teamcode.Core.BetaSorterHardware.BlenderSteps.*;
import static org.firstinspires.ftc.teamcode.Core.BetaSorterHardware.FeederState.*;
import static org.firstinspires.ftc.teamcode.Core.SorterHardware.positionState.FIRE;
import static org.firstinspires.ftc.teamcode.Core.SorterHardware.positionState.LOAD;

public class BetaSorterHardware extends SorterHardware {

    public Robot robot;
    public BetaSorterHardware(Robot robot) {
        super(robot);
        this.robot = super.disRobot;
    }

    private boolean tryToMove = false;
    private boolean doneMoving = false;

    enum BlenderSteps {
        READY_FOR_COMMANDS,
        STALLING_UNTIL_SAFE_OR_NEEDED, CHECK_IF_SAFE, MOVING, RESET,
        INTAKE, OUTTAKE
    }
    private BlenderSteps currentBlenderStep = READY_FOR_COMMANDS;

    enum FeederState {PASSIVE, ROTATE, INTAKE, REVERSE}
    private FeederState currentFeederState = PASSIVE;
    private int ensureBlenderPosition = 0;
    @Override
    public void updateSorterHardware() {
        robot.telemetry.addLine("Untested sorter hardware, I choose you!");
        robot.telemetry.addData("Blender step", currentBlenderStep);
        switch (currentBlenderStep) {
            case READY_FOR_COMMANDS:
                if (tryToMove) {
                    tryToMove = false;
                    doneMoving = false;
                    nextStep(STALLING_UNTIL_SAFE_OR_NEEDED);
                }
                break;
            case STALLING_UNTIL_SAFE_OR_NEEDED:
                if (closedCheck() && legalToSpin) {
                    nextStep(MOVING);
                }
                if (this.positionedCheck()) {
                    nextStep(RESET);
                }
                break;
            case CHECK_IF_SAFE:
                break;
            case MOVING:
                setFeeders(ROTATE);
                spin();
                if (this.positionedCheck()) {
                    ensureBlenderPosition += 1;
                }
                if (ensureBlenderPosition >= 10) {
                    nextStep(RESET);
                }
                break;
            case RESET:
                ensureBlenderPosition = 0;
                doneMoving = true;
                setFeeders(PASSIVE);
                nextStep(READY_FOR_COMMANDS);
                break;
        }

        switch (currentFeederState) {
            case INTAKE:
                runFeeders(feederIntakeSpeed);
                break;
            case ROTATE:
                runFeeders(feederRotateSpeed);
                break;
            case REVERSE:
                runFeeders(-1);
                break;
            case PASSIVE:
                runFeeders(passiveFeederSpeed);
                break;
        }

        updateState();
    }
    private void nextStep(BlenderSteps nextStep) {
        currentBlenderStep = nextStep;
    }

    public boolean doneMoving() {
        if (doneMoving) {
            doneMoving = false;
            return true;
        }
        else return false;


    }

    @Override
    public void prepareNewMovement(int targetTickPose) {
        this.prepareNewMovement(motor.getCurrentPosition(), targetTickPose);
    }

    @Override
    public void prepareNewMovement(int currentTickPose, int targetTickPose) {
        lastSafePosition = currentTickPose;
        reference = (findFastestRotationInTicks(currentTickPose, targetTickPose));
        tryToMove = true;
    }

    public void intake() {

    }

    @Override
    public boolean positionedCheck() {
        int currentMotorPosition = motor.getCurrentPosition();

        if(currentMotorPosition > reference - tickTolerance && currentMotorPosition < reference + tickTolerance)
        {
            currentlyMoving = false;
            return true;
        }
        else return false;
    }

    private void setFeeders(FeederState newState) {
        currentFeederState = newState;
    }

    private void updateState() {
        switch (disRobot.sorterLogic.getCurrentOffset()) {
            // Firing positions
            case 1:
            case 3:
            case 5:
                currentPositionState = FIRE;
                break;

            // Loading positions
            case 0:
            case 2:
            case 4:
                currentPositionState = LOAD;
                break;

            // Not in a position (-1)
            default: currentPositionState = positionState.SWITCH;
        }
    }
}
