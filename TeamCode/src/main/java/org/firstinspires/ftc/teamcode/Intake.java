package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


/**
 *    Everything related to the intake motors and servos to open/close
 */
public class Intake {

    // Hardware
    DcMotor intakeLeftMotor   = null;
    DcMotor  intakeRightMotor  = null;
    Servo intakeLeftServo  = null;
    Servo    intakeRightServo = null;

    // Intake constants
    final static double INTAKE_LEFT_HOME = 0.94;
    final static double INTAKE_LEFT_RELEASE = 0.61;
    final static double INTAKE_RIGHT_HOME = 0.11;
    final static double INTAKE_RIGHT_RELEASE = 0.32;
    final static double MAX_IN_POWER = 1.0;
    final static double MIN_IN_POWER = 0.6;
    final static double IN_POWER_DELTA = 0.01;      // Amount to increment/decrement power per cycle

    /* Intake state variables */
    boolean intakeCycle = true;        // True we are incrementing right power and decrementing left
    boolean isIntakeClosed = true;
    boolean isIntakeInOn = false;
    boolean isIntakeOutOn = false;

    // Place to track desired left/right motor power as we cycle them
    double rInPower = 0.0;
    double lInPower = 0.0;

    /**
     * Constructor
     */
    public void Intake() {
        // Do nothing
    }

    /**
     *  Initialize the intake
     *
     * @param hw    Hardwaremap for our robot
     * @param lm    Name of left intake motor
     * @param rm    Name of right intake motor
     * @param ls    Name of left intake arm servo
     * @param rs    Name of right intake arm servo
     */
    public void init (HardwareMap hw, String lm, String rm, String ls, String rs) {
        // Define and Initialize intake Motors
        intakeLeftMotor = hw.dcMotor.get(lm);
        intakeRightMotor = hw.dcMotor.get(rm);
        intakeLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeRightMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set their operating modes and stop them
        intakeLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setStop();

        // Define and initialize Intake servos
        intakeLeftServo = hw.servo.get(ls);
        intakeRightServo = hw.servo.get(rs);
        setClosed();
    }

    /**
     *   Open the intake
     */
    public void setOpen() {
        intakeLeftServo.setPosition(INTAKE_LEFT_RELEASE);
        intakeRightServo.setPosition(INTAKE_RIGHT_RELEASE);
        isIntakeClosed = false;
    }

    /**
     *   Close the intake
     */
    public void setClosed()
    {
        intakeLeftServo.setPosition(INTAKE_LEFT_HOME);
        intakeRightServo.setPosition(INTAKE_RIGHT_HOME);
        isIntakeClosed = true;
    }

    /**
     *   Set the intake feed wheels in
     */
    public void setIn() {
        rInPower = MIN_IN_POWER;
        lInPower = MAX_IN_POWER;
        intakeLeftMotor.setPower(lInPower);
        intakeRightMotor.setPower(rInPower);
        isIntakeInOn = true;
        isIntakeOutOn = false;
        intakeCycle = true;
    }

    /**
     * Process 1 cycle of varying the intake motor powers.
     *
     * Only works if we are feeding in, not out.   Cycles the power between min and max values.
     */
    public void updateInPower() {
        // Does nothing unless we are currently intaking
        if (isIntakeInOn) {
            if (intakeCycle) {
                // In 1st half of cycle
                rInPower = Math.max(rInPower+IN_POWER_DELTA, MAX_IN_POWER);
                lInPower = Math.min(lInPower-IN_POWER_DELTA, MIN_IN_POWER);
                if (rInPower > (MAX_IN_POWER - IN_POWER_DELTA)) {
                    // Reached end so reverse our direction
                    intakeCycle = false;
                }
                else {
                    // In 2nd half of cycle
                    lInPower = Math.max(lInPower+IN_POWER_DELTA, MAX_IN_POWER);
                    rInPower = Math.min(rInPower-IN_POWER_DELTA, MIN_IN_POWER);
                    if (lInPower > (MAX_IN_POWER - IN_POWER_DELTA)) {
                        // Reached end so reverse our direction
                        intakeCycle = true;
                    }
                }

            }
        }
    }

    /**
     *  Stop the intake feed wheels
     */
    public void setStop() {
        intakeLeftMotor.setPower(0.0);
        intakeRightMotor.setPower(0.0);
        isIntakeInOn = false;
        isIntakeOutOn = false;
    }

    /**
     * Feed the intake in reverse
     */
    public void setOut() {
        intakeLeftMotor.setPower(-1.0);
        intakeRightMotor.setPower(-1.0);
        isIntakeInOn = false;
        isIntakeOutOn = true;
    }

    /**
     *
     * @return  true if intake is in closed position
     */
    public boolean isClosed() {
        return isIntakeClosed;
    }

    /**
     *
     * @return  true if we are currently feeding in direction
     */
    public boolean isIn() {
        return isIntakeInOn;
    }

    /**
     *
     * @return true if we are currently feeding out direction
     */
    public boolean isOut() {
        return isIntakeOutOn;
    }

    /**
     *
     * This is designed to be used for manual tuning of servo Min/Max constants not for routine use
     *
     * @return  current left servo position
     */
    public double getLeftServoPos() {
        return intakeLeftServo.getPosition();
    }

    /**
     *
     * This is designed to be used for manual tuning of servo Min/Max constants not for routine use
     *
     * @return  current right servo position
     */

    public double getRightServoPos() {
        return intakeRightServo.getPosition();
    }

    /**
     *
     * This is designed to be used for manual tuning of servo Min/Max constants not for routine use
     *
     * @param pos  desired left servo position
     */
    public void setLeftServoPos(double pos) {
        pos = Range.clip(pos, 0.0, 1.0);
        intakeLeftServo.setPosition(pos);
    }

    /**
     *
     * This is designed to be used for manual tuning of servo Min/Max constants not for routine use
     *
     * @param pos  desired right servo position
     */

    public void setRightServoPos(double pos) {
        pos = Range.clip(pos, 0.0, 1.0);
        intakeRightServo.setPosition(pos);
    }
}
