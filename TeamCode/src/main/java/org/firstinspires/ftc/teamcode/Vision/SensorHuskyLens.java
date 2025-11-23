/*
Copyright (c) 2023 FIRST

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.Vision;

import static com.qualcomm.hardware.dfrobot.HuskyLens.Algorithm.*;

import static java.util.concurrent.TimeUnit.*;

import androidx.annotation.Discouraged;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.Core.ArtifactLocator;
import org.firstinspires.ftc.teamcode.Core.Robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.concurrent.TimeUnit;

/*
 * This OpMode illustrates how to use the DFRobot HuskyLens.
 *
 * The HuskyLens is a Vision Sensor with a built-in object detection model.  It can
 * detect a number of predefined objects and AprilTags in the 36h11 family, can
 * recognize colors, and can be trained to detect custom objects. See this website for
 * documentation: https://wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336
 *
 * For detailed instructions on how a HuskyLens is used in FTC, please see this tutorial:
 * https://ftc-docs.firstinspires.org/en/latest/devices/huskylens/huskylens.html
 * 
 * This sample illustrates how to detect AprilTags, but can be used to detect other types
 * of objects by changing the algorithm. It assumes that the HuskyLens is configured with
 * a name of "huskylens".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@Disabled
@TeleOp(name = "Sensor: HuskyLens", group = "Sensor")
public class SensorHuskyLens extends LinearOpMode {

    private final int READ_PERIOD = 1;
    private final HuskyLens.Algorithm currentAlgorithm = OBJECT_CLASSIFICATION;
    Robot robot;

    private HuskyLens huskyLens;
    //scanBox[] boxes = new scanBox[6];


    public SensorHuskyLens(Robot robot)
    {
        this.robot = robot;
        huskyLens = robot.husky;
        huskyLens.selectAlgorithm(currentAlgorithm);
        Deadline rateLimit = new Deadline(READ_PERIOD, SECONDS);
        rateLimit.expire();
    }


    @Deprecated
    public void runOpMode()
    {
        huskyLens = robot.husky;

        /*
         * This sample rate limits the reads solely to allow a user time to observe
         * what is happening on the Driver Station telemetry.  Typical applications
         * would not likely rate limit.
         */
        Deadline rateLimit = new Deadline(READ_PERIOD, SECONDS);

        /*
         * Immediately expire so that the first time through we'll do the read.
         */
        rateLimit.expire();

        /*
         * Basic check to see if the device is alive and communicating.  This is not
         * technically necessary here as the HuskyLens class does this in its
         * doInitialization() method which is called when the device is pulled out of
         * the hardware map.  However, sometimes it's unclear why a device reports as
         * failing on initialization.  In the case of this device, it's because the
         * call to knock() failed.
         */
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }


        huskyLens.selectAlgorithm(currentAlgorithm);

        telemetry.update();
        waitForStart();

        /*
         * Looking for AprilTags per the call to selectAlgorithm() above.  A handy grid
         * for testing may be found at https://wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336#target_20.
         *
         * Note again that the device only recognizes the 36h11 family of tags out of the box.
         */
        while(opModeIsActive()) {
            if (!rateLimit.hasExpired()) {
                continue;
            }
            rateLimit.reset();

            /*
             * All algorithms, except for LINE_TRACKING, return a list of Blocks where a
             * Block represents the outline of a recognized object along with its ID number.
             * ID numbers allow you to identify what the device saw.  See the HuskyLens documentation
             * referenced in the header comment above for more information on IDs and how to
             * assign them to objects.
             *
             * Returns an empty array if no objects are seen.
             */
            ArrayList<HuskyLens.Block> blocks =
                    (ArrayList<HuskyLens.Block>) Arrays.asList(huskyLens.blocks());
            telemetry.addData("Block count", blocks.size());
            for (HuskyLens.Block currentBlock : blocks) {
                telemetry.addData("Block", currentBlock.toString());
                /*
                 * Here inside the FOR loop, you could save or evaluate specific info for the currently recognized Bounding Box:
                 * - blocks[i].width and blocks[i].height   (size of box, in pixels)
                 * - blocks[i].left and blocks[i].top       (edges of box)
                 * - blocks[i].x and blocks[i].y            (center location)
                 * - blocks[i].id                           (Color ID)
                 *
                 * These values have Java type int (integer).
                 */
                if (currentBlock.y > 160) {
                    blocks.remove(currentBlock);
                } else if (currentBlock.x < 40 || currentBlock.x > 280) {
                    blocks.remove(currentBlock);
                }
                //checkZone(blocks[i]);
            }

            robot.sorterLogic.sortOutBlobs(blocks.get(0).id);


            telemetry.update();
        }
    }

    @Deprecated
    void checkZone(HuskyLens.Block blockData) {
        for(ArtifactLocator.Zone checkingZone : robot.sorterLogic.allZones) {
            if((checkingZone.inRange(blockData.x, blockData.y))) {
                robot.sorterLogic.sortOutBlobs(blockData.id);
                doTele(robot.sorterLogic.allZones.indexOf(checkingZone), blockData);
            }
            else {
                robot.sorterLogic.sortOutBlobs(0);
            }
        }
    }

    public void updateBlockScan()
    {
        HuskyLens.Block[] blocks = huskyLens.blocks();
        ArrayList<HuskyLens.Block> filteredBlocks = new ArrayList<>();
        robot.telemetry.addData("Unfiltered Block count", blocks.length);
        for (HuskyLens.Block currentBlock : blocks) {
            robot.telemetry.addData("Block", currentBlock.toString());
            /*
             * Here inside the FOR loop, you could save or evaluate specific info for the currently recognized Bounding Box:
             * - blocks[i].width and blocks[i].height   (size of box, in pixels)
             * - blocks[i].left and blocks[i].top       (edges of box)
             * - blocks[i].x and blocks[i].y            (center location)
             * - blocks[i].id                           (Color ID)
             *
             * These values have Java type int (integer).
             */
            if (currentBlock.y > 160) {
                continue;
            } else if (currentBlock.x < 40 || currentBlock.x > 280) {
                continue;
            }
            filteredBlocks.add(currentBlock);
        }

        if (filteredBlocks.isEmpty()) {
            robot.telemetry.addData("Searching for blocks", "None found");
            return;
        }
        int foundID = filteredBlocks.get(0).id;
        robot.sorterLogic.sortOutBlobs(foundID);
        robot.telemetry.addData("Searching for blocks", "ID is " + foundID);
    }

    public class scanBox
    {
        public scanBox(int minX, int maxX, int minY, int maxY) {
            this.minX = minX;
            this.maxX = maxX;
            this.minY = minY;
            this.maxY = maxY;
        }

        public int maxX;
        public int maxY;
        public int minX;
        public int minY;
    }

    public void startHusky()
    {
        huskyLens.selectAlgorithm(currentAlgorithm);
    }

    public void doTele(int i, HuskyLens.Block blockData)
    {
        telemetry.addData("In Zone:"+i, "");
        telemetry.addData("Color ID Detected: ", blockData.id);
    }

}