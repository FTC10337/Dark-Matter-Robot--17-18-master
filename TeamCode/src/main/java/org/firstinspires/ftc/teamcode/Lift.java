
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 *    Everything related to the lift motor
 */
public class Lift {

    // Hardware
    public DcMotor liftMotor = null;
    public DigitalChannel liftLimitB = null;
    public DigitalChannel liftLimitT = null;

    public int targetPos = 0;
    boolean runToPos = false;

    public int LIFT_TIME = 3000;
    ElapsedTime liftTimer = new ElapsedTime();

    /* Lift constants */
    static final double     LIFT_POWER = 1.0;
    static final int        LIFT_COUNTS_PER_MOTOR_REV    = 7 ;    // Neverrest
    static final double     LIFT_DRIVE_GEAR_REDUCTION    = 20; // Neverest 20:1
    static final double     LIFT_PULLEY_DIAMETER_INCHES   = 0.955;     // For figuring circumference
    static final double     LIFT_COUNTS_PER_INCH         = (4 * LIFT_COUNTS_PER_MOTOR_REV * LIFT_DRIVE_GEAR_REDUCTION) /
            (LIFT_PULLEY_DIAMETER_INCHES * 3.1415);


    // Lift variables
    public int LIFT_TOP_POS = (int) (12.75*LIFT_COUNTS_PER_INCH);
    public int LIFT_MID_POS = (int) (7*LIFT_COUNTS_PER_INCH);
    public int LIFT_BTM_POS = (int) (0.5*LIFT_COUNTS_PER_INCH);



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
        targetPos = LIFT_TOP_POS;
        liftMotor.setTargetPosition(targetPos);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(LIFT_POWER);
        runToPos = true;
        liftTimer.reset();
    }

    // Set lift position to middle
    public void setLiftMid() {
        targetPos = LIFT_MID_POS;
        liftMotor.setTargetPosition(targetPos);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(LIFT_POWER);
        runToPos = true;
        liftTimer.reset();
    }

    // Set lift position to bottom
    public void setLiftBtm() {
        targetPos = LIFT_BTM_POS;
        liftMotor.setTargetPosition(targetPos);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(LIFT_POWER);
        runToPos = true;
        liftTimer.reset();
    }

    // Hard Stop Lift
    public void stopLift() {
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setPower(0.0);
    }


    // Move lift to updated lift position then stop
    public void updateLiftMotor() {

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
            liftTimer.reset();
            return true;
        }
    }

    // Resets lift encoder to TOP pos using lift limit switch
    public boolean resetTopPos() {
        if (liftLimitT.getState()) {
            liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftMotor.setPower(0.50);
            return false;
        } else {
            liftMotor.setPower(0.0);
            //liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftTimer.reset();
            return true;
        }
    }

    public boolean isResetComplete() {
        if (liftMotor.getMode() == DcMotor.RunMode.STOP_AND_RESET_ENCODER && liftTimer.milliseconds() > 1000) {
            return true;
        } else return false;
    }

    public boolean isMoving() {
        if (liftMotor.isBusy() && liftTimer.milliseconds() < LIFT_TIME) {
            return false;
        } else {
            stopLift();         // Reached end or expired so force stop motor for safety
            return true;
        }
    }

    public boolean reachedFloor() {
        return (Math.abs(liftMotor.getCurrentPosition() - targetPos) < 0.5 * LIFT_COUNTS_PER_INCH);
    }

}


