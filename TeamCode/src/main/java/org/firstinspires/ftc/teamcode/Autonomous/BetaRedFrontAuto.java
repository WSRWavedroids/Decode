package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * This is a shell that sets the alliance key to "RED" and runs BetaBlueFrontAuto with it. It's
 * efficient, at least. But this class is basically just a button on the control hub. The auto is
 * in BetaBlueFrontAuto. Your problem is probably there.
 */
@Autonomous(group = "Basic", name = "BETA Red Front")
public class BetaRedFrontAuto extends BetaBlueFrontAuto {

    public void init() {
        super.init();
        // Override the alliance key set in the blue auto to run the red one
        blackboard.put(ALLIANCE_KEY, "RED");
    }

    public void init_loop() {
        super.init_loop();
    }

    public void start() {
        super.start();
    }

    public void loop() {
        super.loop();
    }

    public void stop() {
        super.stop();
    }
}


