package org.firstinspires.ftc.teamcode.Core;

import static org.firstinspires.ftc.teamcode.Core.ArtifactLocator.slotState.EMPTY;
import static org.firstinspires.ftc.teamcode.Core.Robot.openClosed.CLOSED;
import static org.firstinspires.ftc.teamcode.Core.Robot.openClosed.OPEN;

import android.telephony.IccOpenLogicalChannelResponse;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    int currentSlot = 0;
    queueBall[] balls;

    public fireQueue(Robot robotFile) {
        robot = robotFile;
        sorterHardware = robot.sorterHardware;
        sorterLogic = robot.sorterLogic;
        launcherHardware = robot.launcher;
        inventoryCam = robot.inventoryCam;


        balls = new queueBall[3];

    }

    public void addToNextSpot(ArtifactLocator.slotState color)
    {
        if(!(currentSlot++ > 3))
        {
            currentSlot++;
            balls[currentSlot].color = color;
        }
    }

    public void addToListDirectly(int positionInList, ArtifactLocator.slotState color)
    {
        balls[positionInList].color = color;
    }

    public void clearList()
    {
        currentSlot = 0;
        for(int i = 0; i < 3; i++)
        {
            balls[i].color = EMPTY;
        }
        firstFired = false;
        secondFired = false;
        thirdFired = false;
    }

    public void fireAll(double speedTarget)
    {
        if(wantToFireQueue && !launcherHardware.waitingToFire && sorterHardware.fireSafeCheck())
        {
            if(!firstFired)
            {
                firstFired = true;
                sorterHardware.prepareNewMovement(sorterHardware.motor.getCurrentPosition(), sorterLogic.findFirstType(balls[0].color).getFirePosition());
                launcherHardware.readyFire(speedTarget);
            }
            else if(firstFired && !secondFired)
            {
                secondFired = true;
                sorterHardware.prepareNewMovement(sorterHardware.motor.getCurrentPosition(), sorterLogic.findFirstType(balls[1].color).getFirePosition());
                launcherHardware.readyFire(speedTarget);
            }
            else if(firstFired && secondFired && !thirdFired)
            {
                thirdFired = true;
                sorterHardware.prepareNewMovement(sorterHardware.motor.getCurrentPosition(), sorterLogic.findFirstType(balls[2].color).getFirePosition());
                launcherHardware.readyFire(speedTarget);
            }
            else
            {
                clearList();
                wantToFireQueue = false;
                launcherHardware.setLauncherSpeed(0);
            }
        }

    }
}

class queueBall
{
    ArtifactLocator.slotState color;
}




