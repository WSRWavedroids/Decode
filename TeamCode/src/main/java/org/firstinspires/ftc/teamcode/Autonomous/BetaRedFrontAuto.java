package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Autonomous.BetaBlueFrontAuto.step.CHECK_MOVE_1;
import static org.firstinspires.ftc.teamcode.Autonomous.BetaBlueFrontAuto.step.CHECK_MOVE_2;
import static org.firstinspires.ftc.teamcode.Autonomous.BetaBlueFrontAuto.step.CHECK_TAG;
import static org.firstinspires.ftc.teamcode.Autonomous.BetaBlueFrontAuto.step.FINE_TUNE_TARGETING;
import static org.firstinspires.ftc.teamcode.Autonomous.BetaBlueFrontAuto.step.FIRE_FIRST_PATTERN;
import static org.firstinspires.ftc.teamcode.Autonomous.BetaBlueFrontAuto.step.FIRST_SPIN;
import static org.firstinspires.ftc.teamcode.Autonomous.BetaBlueFrontAuto.step.LAUNCHER_ON;
import static org.firstinspires.ftc.teamcode.Autonomous.BetaBlueFrontAuto.step.RESET_BLENDER;
import static org.firstinspires.ftc.teamcode.Autonomous.BetaBlueFrontAuto.step.SET_APRILTAG_PIPELINE;
import static org.firstinspires.ftc.teamcode.Autonomous.BetaBlueFrontAuto.step.START;
import static org.firstinspires.ftc.teamcode.Autonomous.BetaBlueFrontAuto.step.TURN_BACK_TOWARDS_GOAL;
import static org.firstinspires.ftc.teamcode.Autonomous.BetaBlueFrontAuto.step.UNPARK_1;
import static org.firstinspires.ftc.teamcode.Autonomous.BetaBlueFrontAuto.step.UNPARK_2;
import static org.firstinspires.ftc.teamcode.Autonomous.BetaBlueFrontAuto.step.UNPARK_3;
import static org.firstinspires.ftc.teamcode.Autonomous.BetaBlueFrontAuto.step.YAY;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.Robot;

/**
 * This file is our iterative (Non-Linear) "OpMode" for TeleOp.
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is selected on the Robot Controller and executed.
 * This particular one is called "Lean Mean TeleOp Machine". I had a little too much fun with naming this.
 * <p>
 * This OpMode controls the functions of the robot during the driver-controlled period.
 * <p>
 * If the "@Disabled" line is not commented out, the program will not show up on the driver hub.
 * If you ever have problems with the program not showing up on the driver hub, it's probably because of that.
 * <p>
 * Throughout this program, there are comments explaining what everything does because previous programmers
 * did a horrible job of doing that.
 */
@Autonomous(group = "Basic", name = "BETA Red Front")
public class BetaRedFrontAuto extends BetaBlueFrontAuto {

    // This section tells the program all of the different pieces of hardware that are on our robot that we will use in the program.
    /*
     * Code to run ONCE when the driver hits INIT
     */
    public void init() {
        super.init();
        blackboard.put(ALLIANCE_KEY, "RED");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    public void init_loop() {
        super.init_loop();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    public void start() {
        super.start();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    public void loop() {
        super.loop();
    }

    /**
     * Code to run ONCE after the driver hits STOP
     */
    public void stop() {
        super.stop();
    }
}


