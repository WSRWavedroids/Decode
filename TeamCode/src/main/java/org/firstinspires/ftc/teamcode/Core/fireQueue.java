package org.firstinspires.ftc.teamcode.Core;

import static org.firstinspires.ftc.teamcode.Core.ArtifactLocator.slotState.EMPTY;
import static org.firstinspires.ftc.teamcode.Core.ArtifactLocator.slotState.GREEN;
import static org.firstinspires.ftc.teamcode.Core.ArtifactLocator.slotState.PURPLE;
import static org.firstinspires.ftc.teamcode.Core.ArtifactLocator.slotState.UNKNOWN;
import static org.firstinspires.ftc.teamcode.Core.Robot.openClosed.CLOSED;
import static org.firstinspires.ftc.teamcode.Core.Robot.openClosed.OPEN;

import android.telephony.IccOpenLogicalChannelResponse;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.Vision.SensorHuskyLens;


@Configurable
public class fireQueue {
    ///This file will hopefully allow us to queue three shots back to back, eventually allowing
    ///caden to fire back to back to back
    ///also this could clean up autonomous firing sequences a bit

    private Robot robot;
    private SorterHardware sorterHardware;
    private ArtifactLocator sorterLogic;
    private LauncherHardware launcherHardware;
    private SensorHuskyLens inventoryCam;

    private boolean firstFired = false;
    private boolean secondFired = false;
    private boolean thirdFired = false;
    public boolean wantToFireQueue = false;

    public boolean noBallsQueued = true;
    int currentSlot = 0;
    public queueBall[] balls;

    public fireQueue(Robot robotFile) {
        robot = robotFile;
        sorterHardware = robot.sorterHardware;
        sorterLogic = robot.sorterLogic;
        launcherHardware = robot.launcher;
        inventoryCam = robot.inventoryCam;

        balls = new queueBall[3];

        for(int i = 0; i < 3; i++)
        {
           balls[i] = new queueBall();
           balls[i].color = EMPTY;
        }

    }

    public void addToNextSpotColor(ArtifactLocator.slotState color)
    {
        if(currentSlot < 3)
        {
            balls[currentSlot].color = color;
            currentSlot++;
            noBallsQueued = false;
        }
    }

    public void addToNextSpotSimple()
    {
        if(currentSlot < 3)
        {
            noBallsQueued = false;
            balls[currentSlot].color = UNKNOWN;
            currentSlot++;
        }
    }

    public void addToListDirectly(int positionInList, ArtifactLocator.slotState color)
    {
        balls[positionInList].color = color;
        noBallsQueued = false;
    }

    public void clearList()
    {
        noBallsQueued = true;
        currentSlot = 0;
        for(int i = 0; i < 3; i++)
        {
            balls[i].color = EMPTY;
        }
        firstFired = false;
        secondFired = false;
        thirdFired = false;
    }


    public boolean checkForExistingQueue()
    {
        for(int i = 0; i < 3; i++)
        {
            if(!balls[i].color.equals(EMPTY))
            {
                return  noBallsQueued = false;
            }
        }
        return noBallsQueued = true;

    }

    public void fillSimple()
    {
        balls[0].color = UNKNOWN;
        balls[1].color = UNKNOWN;
        balls[2].color = UNKNOWN;
        noBallsQueued = false;
    }

    public void addPattern(Object Pattern)
    {
        if(Pattern == "PGP")
        {
            balls[0].color = PURPLE;
            balls[1].color = GREEN;
            balls[2].color = PURPLE;
            noBallsQueued = false;
        }
        else if(Pattern == "PPG")
        {
            balls[0].color = PURPLE;
            balls[1].color = PURPLE;
            balls[2].color = GREEN;
            noBallsQueued = false;
        }
        else if(Pattern == "GPP")
        {
            balls[0].color = GREEN;
            balls[1].color = PURPLE;
            balls[2].color = PURPLE;
            noBallsQueued = false;
        }
        else
        {
            fillSimple();
        }
    }

    public void fireAllSmart(double speedTarget)
    {
        if(!robot.launcher.waitingToFire && !robot.launcher.onCooldown && !robot.sorterHardware.onCooldown)
        {
            if(!firstFired && !balls[0].color.equals(EMPTY))
            {
                firstFired = true;
                sorterHardware.prepareNewMovement(sorterHardware.motor.getCurrentPosition(), sorterLogic.findFirstType(balls[0].color).getFirePosition());
                launcherHardware.readyFire(speedTarget, true);
            }
            else if(firstFired && !secondFired && !balls[1].color.equals(EMPTY))
            {
                secondFired = true;
                sorterHardware.prepareNewMovement(sorterHardware.motor.getCurrentPosition(), sorterLogic.findFirstType(balls[1].color).getFirePosition());
                launcherHardware.readyFire(speedTarget, true);
            }
            else if(firstFired && secondFired && !thirdFired && !balls[2].color.equals(EMPTY))
            {
                thirdFired = true;
                sorterHardware.prepareNewMovement(sorterHardware.motor.getCurrentPosition(), sorterLogic.findFirstType(balls[2].color).getFirePosition());
                launcherHardware.readyFire(speedTarget, true);
            }
            else if(balls[0].color.equals(EMPTY) && balls[1].color.equals(EMPTY) && balls[2].color.equals(EMPTY))
            {
                clearList();
                wantToFireQueue = false;
                launcherHardware.setLauncherSpeed(0);
                sorterHardware.prepareNewMovement(sorterHardware.motor.getCurrentPosition(), sorterHardware.positions[0]);
            }
        }

    }


    public void fireAllDumb(double speedTarget)
    {
        launcherHardware.setLauncherSpeed(1);
        if(!robot.launcher.waitingToFire && !robot.launcher.onCooldown && !robot.sorterHardware.onCooldown)
        {

            if(!firstFired && !balls[0].color.equals(EMPTY))
            {
                sorterHardware.prepareNewMovement(sorterHardware.motor.getCurrentPosition(), sorterHardware.positions[1]);
                launcherHardware.readyFire(speedTarget, true);
                firstFired = true;
                balls[0].color = (EMPTY);
            }
            else if(firstFired && !secondFired && !balls[1].color.equals(EMPTY))
            {
                sorterHardware.prepareNewMovement(sorterHardware.positions[1], sorterHardware.positions[3]);
                launcherHardware.readyFire(speedTarget, true);
                secondFired = true;
                balls[1].color = (EMPTY);
            }
            else if(firstFired && secondFired && !thirdFired && !balls[2].color.equals(EMPTY))
            {
                sorterHardware.prepareNewMovement(sorterHardware.positions[3], sorterHardware.positions[5]);
                launcherHardware.readyFire(speedTarget, true);
                balls[2].color = (EMPTY);
                thirdFired = true;
            }
            else if(balls[0].color.equals(EMPTY) && balls[1].color.equals(EMPTY) && balls[2].color.equals(EMPTY))
            {
                clearList();
                wantToFireQueue = false;
                launcherHardware.setLauncherSpeed(0);
                sorterHardware.prepareNewMovement(sorterHardware.motor.getCurrentPosition(), sorterHardware.positions[0]);
            }
        }

    }
}

class queueBall
{
    ArtifactLocator.slotState color;
}




