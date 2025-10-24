package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.AutonomousPLUS;
import org.firstinspires.ftc.teamcode.LauncherHardware;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.SorterHardware;
import org.firstinspires.ftc.teamcode.Vision.Limelight_Randomization_Scanner;

@Autonomous(group = "Basic", name = "Blue Back Start")
public class Test_Randomization_Blue extends AutonomousPLUS {

    public Limelight_Randomization_Scanner Limelight = new Limelight_Randomization_Scanner();
    public Limelight_Target_Scanner scanner = new Limelight_Target_Scanner();
    public String currentPosition;
    public String pattern;

    public static final String ALLIANCE_KEY = "Alliance";
    public static final String PATTERN_KEY = "Pattern";

    public WaveTag targetData = null;
    public LauncherHardware launcher;
    public SorterHardware sorter;

    private Robot robot;

    public void runOpMode() {

        super.runOpMode();

        robot = new Robot(hardwareMap, telemetry, this);
        targetData = robot.targetTag;
        launcher = robot.launcher;
        sorter = robot.sorterHardware;



        if(opModeInInit())
        {
            prepareAuto();
            Limelight.InitLimeLight(0, robot.hardwareMap);
            blackboard.put(ALLIANCE_KEY, "BLUE");
            while(opModeInInit())
            {
                pattern = Limelight.GetRandomization();
                telemetry.addData(pattern, " Works!");
                telemetry.update();

            }
        }

        waitForStart();
        telemetry.addData("Our pattern is: ", pattern, " ...yay");

        if(pattern.equals("PPG"))
        {
            telemetry.addData("We doin", " PPG now");
            blackboard.put(PATTERN_KEY, "PPG");
        }
        else if(pattern.equals("GPP"))
        {
            telemetry.addData("We doin", " GPP now");
            blackboard.put(PATTERN_KEY, "GPP");
        } else if (pattern.equals("PGP"))
        {
            telemetry.addData("We doin", " PGP now");
            blackboard.put(PATTERN_KEY, "PGP");
        }
        else
        {
            telemetry.addData("It failed: ", "Cry Time");
        }
        telemetry.update();


        if (blackboard.get(ALLIANCE_KEY).equals("BLUE")) {
            scanner.InitLimeLightTargeting(2, robot.hardwareMap);
        } else if (blackboard.get(ALLIANCE_KEY).equals("RED")) {
            scanner.InitLimeLightTargeting(1, robot.hardwareMap);
        } else {
            scanner.InitLimeLightTargeting(1, robot.hardwareMap);
        }


        moveXY(450,450,true);
        targetData = scanner.tagInfo();
        if (targetData.currentlyDetected)
        {
            turnRobotLeft((int) (targetData.angleX * (-1660/360)), 1);

        }
        else{
            turnRobotLeft(450, 1);
        }

        launcher.rampSpeed(launcher.findSpeed(targetData.distanceZ));

        int slotOneLaunch = sorter.positions[1];
        int slotTwoLaunch = sorter.positions[3];
        int slotThreeLauch = sorter.positions[5];

        if(pattern.equals("PPG"))
        {
            //Run algorithm to find first purple slot


        }
        else if(pattern.equals("PGP"))
        {
            //Run algorithm to find first purple slot


        }
        else if(pattern.equals("GPP"))
        {

        }
        else//Fire any
        {
            stallTillTrue(robot.sorterHardware.moveSafeCheck()); //Check to see if safe to spin, then do so
            launcher.rampSpeed(launcher.findSpeed(targetData.distanceZ)); //set Motor target speed
            sorter.prepareNewMovement(sorter.motor.getCurrentPosition(), sorter.positions[1]); //command movement
            stallTillTrue(robot.sorterHardware.positionedCheck());
            stallTillTrue(launcher.inSpeedRange);// If we arent at speed yet, stall till we are
            sorter.triggerServo("OPEN");
            stallTillTrue(sorter.openCheck()); // Prepare to fire
            launcher.fire();
            stallTillTrue(!robot.launcher.waitingForServo); //Wait till done firing
            launcher.cutSpeed();// Cut laucher to save power
            sorter.triggerServo("CLOSED"); //Tell to close
            stallTillTrue(sorter.closedCheck()); //Wait for close


            stallTillTrue(robot.sorterHardware.moveSafeCheck()); //Check to see if safe to spin, then do so
            launcher.rampSpeed(launcher.findSpeed(targetData.distanceZ)); //set Motor target speed
            sorter.prepareNewMovement(sorter.motor.getCurrentPosition(), sorter.positions[3]); //command movement
            stallTillTrue(robot.sorterHardware.positionedCheck());
            stallTillTrue(launcher.inSpeedRange);// If we arent at speed yet, stall till we are
            sorter.triggerServo("OPEN");
            stallTillTrue(sorter.openCheck()); // Prepare to fire
            launcher.fire();
            stallTillTrue(!robot.launcher.waitingForServo); //Wait till done firing
            launcher.cutSpeed();// Cut laucher to save power
            sorter.triggerServo("CLOSED"); //Tell to close
            stallTillTrue(sorter.closedCheck()); //Wait for close



            stallTillTrue(robot.sorterHardware.moveSafeCheck()); //Check to see if safe to spin, then do so
            launcher.rampSpeed(launcher.findSpeed(targetData.distanceZ)); //set Motor target speed
            sorter.prepareNewMovement(sorter.motor.getCurrentPosition(), sorter.positions[5]); //command movement
            stallTillTrue(robot.sorterHardware.positionedCheck());
            stallTillTrue(launcher.inSpeedRange);// If we arent at speed yet, stall till we are
            sorter.triggerServo("OPEN");
            stallTillTrue(sorter.openCheck()); // Prepare to fire
            launcher.fire();
            stallTillTrue(!robot.launcher.waitingForServo); //Wait till done firing
            launcher.cutSpeed();// Cut laucher to save power
            sorter.triggerServo("CLOSED"); //Tell to close
            stallTillTrue(sorter.closedCheck()); //Wait for close

            /*//in pedro
            launcher.rampSpeed(launcher.findSpeed(robot.targetTag.distanceZ));
            sorter.prepareNewMovement(sorter.motor.getCurrentPosition(), sorter.positions[1]);
            sorter.triggerServo("OPEN");
            launcher.fire();
            launcher.cutSpeed();// Cut launcher to save power
            sorter.triggerServo("CLOSED"); //Tell to close*/





        }









        sleep(1000000000);

    }
}