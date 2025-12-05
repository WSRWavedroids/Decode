package org.firstinspires.ftc.teamcode.Core;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Iterator;

public class FramerateCalculator {
    private ArrayList<Double> loopTimes = new ArrayList<>();
    private int loopCount = 0;
    private ElapsedTime stopwatch;

    public FramerateCalculator(ElapsedTime stopwatch) {
        this.stopwatch = stopwatch;
    }

    public void update() {
        checkForReset();
        if (loopCount == 0) loopTimes.add(0,0.0);
        loopCount += 1;
        loopTimes.add(loopCount, stopwatch.seconds());
    }

    private void checkForReset() {
        if (loopCount < 2) return;
        if (loopTimes.get(loopCount) < loopTimes.get(loopCount - 1)) {
            loopCount = 0;
            loopTimes = new ArrayList<>();
        }
    }

    private double getFramerate(double timeframe, boolean useTimeframe) {
        ArrayList<Double> tempList = new ArrayList<>(loopTimes);

        double cutoff = stopwatch.seconds() - timeframe;

        Iterator<Double> iterator = tempList.iterator();

        while (iterator.hasNext() && useTimeframe) {
            double checkingTime = iterator.next();

            if (checkingTime < cutoff) {
                iterator.remove();
            } else {
                break;
            }
        }
        if (useTimeframe && stopwatch.seconds() > timeframe) return tempList.size() / timeframe;
        else return tempList.size() / stopwatch.seconds();
    }
    public double getFramerate() {
        return getFramerate(0, false);
    }
    public double getFramerate(double timeframeSeconds) {
        return getFramerate(timeframeSeconds, true);
    }

    public int getLoopCount() {
        return loopCount;
    }
}
