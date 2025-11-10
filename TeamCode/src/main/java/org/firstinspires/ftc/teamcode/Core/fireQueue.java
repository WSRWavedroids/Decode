package org.firstinspires.ftc.teamcode.Core;

import static org.firstinspires.ftc.teamcode.Core.Robot.openClosed.OPEN;

import android.telephony.IccOpenLogicalChannelResponse;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Configurable
public class fireQueue {
    ///This file will hopefully allow us to queue three shots back to back, eventually allowing
    ///caden to fire back to back to back
    ///also this could clean up autonomous firing sequences a bit

    private Robot robot;
    enum color{Purple, Green, Empty};
    int currentSlot = 0;
    queueBall[] balls;

    public fireQueue(Robot robotFile) {
        robot = robotFile;
        balls = new queueBall[3];

    }

    public void addToNextSpot(color color)
    {
        if(!(currentSlot++ > 3))
        {
            currentSlot++;
            balls[currentSlot].thisColor = color;
        }
    }

    public void addToListDirectly(int positionInList, color color)
    {
        balls[positionInList].thisColor = color;
    }

    public void clearList()
    {
        currentSlot = 0;
        for(int i = 0; i < 4; i++)
        {
            balls[i].thisColor = color.Empty;
        }
    }
}

class queueBall
{
    fireQueue.color thisColor;
}




