package org.firstinspires.ftc.teamcode.Core;

import static org.firstinspires.ftc.teamcode.Core.BetaSorterHardware.BlenderSteps.*;
import static org.firstinspires.ftc.teamcode.Core.BetaSorterHardware.FeederState.*;

public class BetaSorterHardware extends SorterHardware {

    public Robot robot;
    public BetaSorterHardware(Robot robot) {
        super(robot);
        this.robot = super.disRobot;
    }

    private boolean tryToMove = false;

    enum BlenderSteps {
        READY_FOR_COMMANDS,
        STALLING_UNTIL_SAFE, CHECK_IF_SAFE, MOVING, RESET,
        INTAKE, OUTTAKE
    }
    private BlenderSteps currentBlenderStep = READY_FOR_COMMANDS;

    enum FeederState {PASSIVE, ROTATE, INTAKE, REVERSE}
    private FeederState currentFeederState = PASSIVE;
    @Override
    public void updateSorterHardware() {
        robot.telemetry.addLine("Untested sorter hardware, I choose you!");
        switch (currentBlenderStep) {
            case READY_FOR_COMMANDS:
                if (tryToMove) {
                    tryToMove = false;
                    nextStep(STALLING_UNTIL_SAFE);
                }
                break;
            case STALLING_UNTIL_SAFE:
                if (moveSafeCheck()) {
                    nextStep(MOVING);
                }
                break;
            case CHECK_IF_SAFE:
                break;
            case MOVING:
                setFeeders(ROTATE);
                spin();
                if (this.positionedCheck()) {
                    nextStep(RESET);
                }
                break;
            case RESET:
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



    }
    private void nextStep(BlenderSteps nextStep) {
        currentBlenderStep = nextStep;
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
}
