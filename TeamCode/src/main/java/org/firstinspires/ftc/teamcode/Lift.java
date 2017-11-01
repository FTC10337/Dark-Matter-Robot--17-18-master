

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 *    Everything related to the lift motor
 */
public class Lift {

    // Hardware
    public DcMotor liftMotor = null;
    public DigitalChannel liftLimitB = null;
    public DigitalChannel liftLimitT = null;

    public int targetPos = 0;
    boolean runUp = false;
    boolean runDown = false;

    public int LIFT_TIME = 2000;
    ElapsedTime liftTimer = new ElapsedTime();

    /* Lift constants */
    static final double     LIFT_POWER = 1.0;
    static final int        LIFT_COUNTS_PER_MOTOR_REV    = 7 ;    // Neverrest
    static final double     LIFT_DRIVE_GEAR_REDUCTION    = 20; // Neverest 20:1
    static final double     LIFT_PULLEY_DIAMETER_INCHES   = 0.955;     // For figuring circumference
    static final double     LIFT_COUNTS_PER_INCH         = (4 * LIFT_COUNTS_PER_MOTOR_REV * LIFT_DRIVE_GEAR_REDUCTION) /
            (LIFT_PULLEY_DIAMETER_INCHES * 3.1415);


    // Lift variables
    public final int LIFT_TOP_POS = (int) (12.75*LIFT_COUNTS_PER_INCH);
    public final int LIFT_MID_POS = (int) (7*LIFT_COUNTS_PER_INCH);
    public final int LIFT_BTM_POS = (int) (0.5*LIFT_COUNTS_PER_INCH);

    public final int LIFT_TOP_OFFSET = 2300;       // Encoder reading when we hit top limit switch
    public final int LIFT_BTM_OFFSET = 0;
    public int liftOffset = LIFT_BTM_OFFSET;                     // Normal position is we start at bottom

    // Timer to tell if intake is still opening/closing
    ElapsedTime timer = new ElapsedTime();

    /**
     * Constructor
     */
    public void Lift() {
        // Do nothing
    }


    /**
     *  Initialize the intake
     *
     * @param hw    Hardwaremap for our robot
     * @param lift  Name of lift motor
     */

    public void init (HardwareMap hw, String lift, String liftLimitBtm, String liftLimitTop) {
        // Define and Initialize intake Motors
        liftMotor = hw.dcMotor.get(lift);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Tune the motor PID parameters
        if (liftMotor instanceof DcMotorEx) {
            DcMotorEx theLift = (DcMotorEx) liftMotor;
            PIDCoefficients pid = theLift.getPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
            // Do any needed PID value adjustments here
            theLift.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pid);

        }


        // Set their operating modes and stop them
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setPower(0.0);

        // Define lift limit switch
        liftLimitB = hw.get(DigitalChannel.class, liftLimitBtm);
        liftLimitT = hw.get(DigitalChannel.class, liftLimitTop);

        // set the digital channel to input.
        liftLimitB.setMode(DigitalChannel.Mode.INPUT);
        liftLimitT.setMode(DigitalChannel.Mode.INPUT);

    }

    // Set lift position to top
    public void setLiftTop() {
        targetPos = LIFT_TOP_POS - liftOffset;
        liftMotor.setTargetPosition(targetPos);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(LIFT_POWER);
        runUp = true;
        liftTimer.reset();
    }

    // Set lift position to middle
    public void setLiftMid() {
        targetPos = LIFT_MID_POS - liftOffset;
        liftMotor.setTargetPosition(targetPos);

        if (liftMotor.getCurrentPosition() < targetPos) {
            runUp = true;
        } else {
            runDown = true;
        }

        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double thePower = LIFT_POWER;
        if (targetPos < liftMotor.getCurrentPosition()){
            thePower /= 1;              // Moving down so use less power
        }
        liftMotor.setPower(thePower);
        liftTimer.reset();
    }

    // Set lift position to bottom
    public void setLiftBtm() {
        targetPos = LIFT_BTM_POS - liftOffset;
        liftMotor.setTargetPosition(targetPos);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double thePower = LIFT_POWER;
        if (targetPos < liftMotor.getCurrentPosition()){
            thePower /= 1;              // Moving down so use less power
        }
        liftMotor.setPower(thePower);
        runDown = true;
        liftTimer.reset();
    }

    public void setLiftHeight(double height) {
        height = Range.clip(height, 1.0, 11.0);
        targetPos = LIFT_BTM_POS - liftOffset + (int)(height * LIFT_COUNTS_PER_INCH);

        liftMotor.setTargetPosition(targetPos);

        if (liftMotor.getCurrentPosition() < targetPos) {
            runUp = true;
        } else {
            runDown = true;
        }

        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double thePower = LIFT_POWER;
        if (targetPos < liftMotor.getCurrentPosition()){
            thePower /= 1;              // Moving down so use less power
        }
        liftMotor.setPower(thePower);
        liftTimer.reset();
    }

    // Hard Stop Lift
    public void stopLift() {
        runDown = false;
        runUp = false;
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setPower(0.0);
    }

    // Resets lift encoder to 0 using lift limit switch
    public boolean resetFloorPos() {
        if (liftLimitB.getState()) {
            liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftMotor.setPower(-0.25);
            return false;
        } else {
            liftMotor.setPower(0.0);
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftOffset = LIFT_BTM_OFFSET;
            liftTimer.reset();
            return true;
        }
    }

    // Resets lift encoder to TOP pos using lift limit switch
    public boolean resetTopPos() {
        if (liftLimitT.getState()) {
            liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftMotor.setPower(0.75);
            return false;
        } else {
            liftMotor.setPower(0.0);
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftOffset = LIFT_TOP_OFFSET;
            liftTimer.reset();
            return true;
        }
    }

    public double distFromBottom() {
        double trueBottom = LIFT_BTM_POS - liftOffset;      // Current encoder reading of bottom of travel
        return ((liftMotor.getCurrentPosition() - trueBottom) / LIFT_COUNTS_PER_INCH);  // Inches above bottom
    }

    public boolean reachedFloor() {
        if ((runUp && !liftLimitT.getState()) ||
                (runDown && !liftLimitB.getState()) ||
                !liftMotor.isBusy() ||
                (Math.abs(liftMotor.getCurrentPosition()-liftMotor.getTargetPosition()) < 0.5*LIFT_COUNTS_PER_INCH)){
            stopLift();
            return true;
        }
        return false;
    }

}


