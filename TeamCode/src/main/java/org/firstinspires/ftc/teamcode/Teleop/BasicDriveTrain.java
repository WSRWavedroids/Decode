package org.firstinspires.ftc.teamcode.Teleop;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.*;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;

@TeleOp(name = "Literally Just the Drive Train", group = "CompBot")
public class BasicDriveTrain extends OpMode {
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    @Override
    public void init() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");

        frontLeftDrive.setDirection(FORWARD);
        frontRightDrive.setDirection(FORWARD);
        backLeftDrive.setDirection(FORWARD);
        backRightDrive.setDirection(FORWARD);

        frontLeftDrive.setZeroPowerBehavior(BRAKE);
        frontRightDrive.setZeroPowerBehavior(BRAKE);
        backLeftDrive.setZeroPowerBehavior(BRAKE);
        backRightDrive.setZeroPowerBehavior(BRAKE);
    }

    @Override
    public void loop() {
        singleJoystickDrive();
    }

    private void singleJoystickDrive() {
        // We don't really know how this function works, but it makes the wheels drive, so we don't question it.
        // Don't mess with this function unless you REALLY know what you're doing.
        float leftY = this.gamepad1.left_stick_y;
        float rightX = this.gamepad1.right_stick_x;
        float leftX = this.gamepad1.left_stick_x;

        if (leftY < 0.1) leftY = 0;
        if (leftX < 0.1) leftX = 0;
        if (rightX < 0.1) rightX = 0;

        float[] motorPowers = new float[4];

        motorPowers[0] = (leftY + leftX + rightX);//might need inverted back
        motorPowers[1] = (leftY - leftX - rightX);
        motorPowers[2] = (leftY - leftX + rightX);
        motorPowers[3] = (leftY + leftX - rightX);

        float max = getLargestAbsVal(motorPowers);
        if (max < 1) {
            max = 1;
        }

        for (int i = 0; i < motorPowers.length; i++) {
            motorPowers[i] *= (1 / max);

            float abs = Math.abs(motorPowers[i]);
            if (abs < 0.05) {
                motorPowers[i] = 0.0f;
            }
            if (abs > 1.0) {
                motorPowers[i] /= abs;
            }
        }

        setIndividualPowers(motorPowers);
    }

    private float getLargestAbsVal(float[] values){
        // This function does some math!
        float max = 0;
        for (float val : values) {
            if (Math.abs(val) > max) {
                max = Math.abs(val);
            }
        }
        return max;
    }

    private void setIndividualPowers(float[] motorPowers) {
        // This function creates an array so that the function below works.
        // Don't mess with this function unless you know what you're doing.

        if (motorPowers.length != 4) {
            return;
        }
        frontLeftDrive.setMode(RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(RUN_WITHOUT_ENCODER);

        frontLeftDrive.setPower(motorPowers[0]);
        frontRightDrive.setPower(motorPowers[1]);
        backLeftDrive.setPower(motorPowers[2]);
        backRightDrive.setPower(motorPowers[3]);
    }
}