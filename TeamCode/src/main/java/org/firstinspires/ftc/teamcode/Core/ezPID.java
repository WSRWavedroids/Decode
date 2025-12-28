package org.firstinspires.ftc.teamcode.Core;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;


@Configurable
public class ezPID {
    /// A PID controller maintains the speed or position of a motor, allowing for consistent movements
    /// EzPID simplifies custom PID controllers and cleans up usage
    /// To use an ezPID, just make an instance in the script using it and pass in the PIDF values
    /// kneecap values, encoder resolution, and a speed or position mode into the constructor.
    /// Then, just call run inside of another script's loop, using the instance you made there
    ///
    /// This script was written by Clay Kramer, FTC 13206 🌊🤖, 2025

    private Robot robot;
    private DcMotorEx motor;

    private PIDMotorGroup motorGroup;
    public double p;
    public double i;
    public double d;
    public double f;
    public double kneecap;
    public ElapsedTime Timer;
    public double lastError;
    public double integralSum;
    public double ticksPerRotation;
    public double tolerance;
    public boolean withinTolerance;

    public enum movementType{SPEED,POSITION}

    movementType mode;

    int numberOfMotorsInGroup;

    ///
    /**
     * This is the constructor for a regular motor PID.
     * Pass in your values and stuff in your script as so
     * private ezPID singleMotorPID(motor, encoderResolutionValue, p, i, d, f, kneecap, toleranceInTicks, motorMode)
     * @param motorIn The motor controlled by the PID
     * @param ticksPerRotationIN The number of ticks in a full rotation
     * @param inP The P value
     * @param inI The I value
     * @param inD The D value
     * @param inF The F value
     * @param kneecapIN A kneecap value, to limit the total possible power of the motor.
     * @param toleranceIN The tolerance in ticks for the checking function/
     * @param modeIN SPEED mode or POSITION mode.
     */
    public ezPID(DcMotorEx motorIn, int ticksPerRotationIN, double inP, double inI, double inD, double inF, double kneecapIN, double toleranceIN, movementType modeIN) {
        motor = motorIn;
        p = inP;
        i = inI;
        d = inD;
        f = inF;
        kneecap = kneecapIN;
        Timer = new ElapsedTime();
        ticksPerRotation = ticksPerRotationIN;
        mode = modeIN;
        tolerance = toleranceIN;
        withinTolerance = false;

    }

    /// This is the constructor for a motor group
    /// Check out that file's comments to understand setting it up better
    /// To make a PID for the motor group, use the following line
    /// private exPID singleMotorPID(PIDMotorGroup(numberOfMotorsIn, DcMotorEx["motors you want here without the "" "]), encoderResolutionValue, p, i, d, f, kneecap, toleranceInTicks, motorMode)
    public ezPID(PIDMotorGroup motorsIN, int ticksPerRotationIN, Double inP, Double inI, Double inD, Double inF, Double kneecapIN, Double toleranceIN, movementType modeIN) {
        motorGroup = motorsIN;
        p = inP;
        i = inI;
        d = inD;
        f = inF;
        kneecap = kneecapIN;
        Timer = new ElapsedTime();
        ticksPerRotation = ticksPerRotationIN;
        mode = modeIN;
        tolerance = toleranceIN;
        withinTolerance = false;

    }




    public PIDDUMP shareInfo() {
        /// This function dumps it's info for another PID to pick up and use
        /// Only hot-swap PIDs if you understand the logic and what might happen with your motor
        PIDDUMP temp = new PIDDUMP() ;
        temp.lastErrorFromOld = lastError;
        temp.timerFromOld = Timer;
        temp.integralFromOld = integralSum;
        return temp;
    }

    public void grabInfoFromPID(PIDDUMP otherPIDINFO)
    {
        /// This function grabs dumped info to allow for seamless controller switching
        /// Only hot-swap PIDs if you understand the logic and what might happen with your motor
        lastError = otherPIDINFO.lastErrorFromOld;
        Timer = otherPIDINFO.timerFromOld;
        integralSum = otherPIDINFO.lastErrorFromOld;
    }


    
    public void runCalledPID(double reference)
    {
        /// CALL THIS ONCE PER LOOP IN ANOTHER SCRIPT OR MOTOR WON'T MOVE
        /// ONLY USE FOR ONE MOTOR ezPID INSTANCES OR WILL THROW ERRORS!
        /// Use the following line to call this properly FOR ONE MOTOR
        /// PIDINSTANCENAME.runCalledPID(Target position(Ticks) or speed (Ticks/s));

        if(mode == movementType.POSITION)
        {
            // obtain the encoder position
            double encoderPosition = motor.getCurrentPosition();
            // calculate the error
            double error = reference - encoderPosition;

            if(Math.abs(error) > tolerance)
            {
                withinTolerance = false;
            }
            else
            {
                withinTolerance = true;
            }

            double derivative;
            // rate of change of the error
            if(Timer.seconds()!= 0)
            {
                derivative = (error - lastError) / Timer.seconds();
            }
            else
            {
                derivative = (error - lastError);
            }


            double feedforward = f * reference;

            if(Timer.seconds() != 0)
            {
                // sum of all error over time
                integralSum = integralSum + (error * Timer.seconds());
            }

            double out = kneecap * ((p * error) + (i * integralSum) + (d * derivative) + feedforward);

            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(out);

            lastError = error;

            // reset the timer for next time
            Timer.reset();

        }
        else if(mode == movementType.SPEED)
        {
            // obtain the encoder position
            double encoderSpeed = motor.getVelocity();
            // calculate the error
            double error = reference - encoderSpeed;

            if(Math.abs(error) > tolerance)
            {
                withinTolerance = false;
            }
            else
            {
                withinTolerance = true;
            }

            // rate of change of the error
            double derivative = (error - lastError) / Timer.seconds();

            // sum of all error over time
            integralSum = integralSum + (error * Timer.seconds());

            double out = (p * error) + (i * integralSum) + (d * derivative);

            motor.setPower(out);

            lastError = error;

            // reset the timer for next time
            Timer.reset();
        }
    }


public void runCalledPIDGroup(double reference)
{
    /// CALL THIS ONCE PER LOOP IN ANOTHER SCRIPT OR MOTOR WON'T MOVE
    /// ONLY USE FOR ONE MOTOR ezPID INSTANCES OR WILL THROW ERRORS!
    /// Use the following line to call this properly FOR MOTOR GROUPS ONLY
    /// PIDGROUPINSTANCENAME.runCalledPID(Target position(Ticks) or speed (Ticks/s));

        if(mode == movementType.POSITION)
        {

            // obtain the encoder position
            double encoderPosition = motor.getCurrentPosition();
            // calculate the error
            double error = reference - encoderPosition;

            if(Math.abs(error) > tolerance)
            {
                withinTolerance = false;
            }
            else
            {
                withinTolerance = true;
            }

            double derivative;
            // rate of change of the error
            if(Timer.seconds()!= 0)
            {
                derivative = (error - lastError) / Timer.seconds();
            }
            else
            {
                derivative = (error - lastError);
            }


            double feedforward = f * reference;

            if(Timer.seconds() != 0)
            {
                // sum of all error over time
                integralSum = integralSum + (error * Timer.seconds());
            }

            double out = kneecap * ((p * error) + (i * integralSum) + (d * derivative) + feedforward);

            for (int i = 0; i < motorGroup.motorGroup.size(); i++) {
                motorGroup.motorGroup.get(i).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorGroup.motorGroup.get(i).setPower(out);
            }
            lastError = error;

            // reset the timer for next time
            Timer.reset();

        }
        else if(mode == movementType.SPEED)
        {
            // obtain the encoder position
            double encoderSpeed = motor.getVelocity();
            // calculate the error
            double error = reference - encoderSpeed;

            if(Math.abs(error) > tolerance)
            {
                withinTolerance = false;
            }
            else
            {
                withinTolerance = true;
            }

            // rate of change of the error
            double derivative = (error - lastError) / Timer.seconds();

            // sum of all error over time
            integralSum = integralSum + (error * Timer.seconds());

            double out = (p * error) + (i * integralSum) + (d * derivative);

            for (int i = 0; i < motorGroup.motorGroup.size(); i++) {
                motorGroup.motorGroup.get(i).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorGroup.motorGroup.get(i).setPower(out);
            }

            lastError = error;

            // reset the timer for next time
            Timer.reset();
        }
    }
    }


class PIDDUMP {
    ElapsedTime timerFromOld;
    double lastErrorFromOld;
    double integralFromOld;
}












