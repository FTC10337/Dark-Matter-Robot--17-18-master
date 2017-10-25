
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 *    Everything related to the lift motor
 */
public class Lift {

    // Hardware
    public DcMotor liftMotor = null;
    public DigitalChannel liftLimit = null;

    public int targetPos = 0;
    boolean runToPos = false;

    public int LIFT_TIME = 3000;
    ElapsedTime liftTimer = new ElapsedTime();

    /* Lift constants */
    static final int        LIFT_COUNTS_PER_MOTOR_REV    = 7 ;    // Neverrest
    static final double     LIFT_DRIVE_GEAR_REDUCTION    = 40*1.54;
    static final double     LIFT_PULLEY_DIAMETER_INCHES   = 0.955;     // For figuring circumference
    static final double     LIFT_COUNTS_PER_INCH         = (4 * LIFT_COUNTS_PER_MOTOR_REV * LIFT_DRIVE_GEAR_REDUCTION) /
            (LIFT_PULLEY_DIAMETER_INCHES * 3.1415);

    // Lift variables
    public int LIFT_TOP_POS = (int) (13*LIFT_COUNTS_PER_INCH);
    public int LIFT_MID_POS = (int) (7*LIFT_COUNTS_PER_INCH);
    public int LIFT_BTM_POS = 0;

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

    public void init (HardwareMap hw, String lift, String liftlimit) {
        // Define and Initialize intake Motors
        liftMotor = hw.dcMotor.get(lift);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set their operating modes and stop them
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setPower(0.0);

        // Define lift limit switch
        liftLimit = hw.get(DigitalChannel.class, liftlimit);
        // set the digital channel to input.
        liftLimit.setMode(DigitalChannel.Mode.INPUT);

    }

    // Set lift position to top
    public void setLiftTop() {
        targetPos = LIFT_TOP_POS;
        liftMotor.setTargetPosition(targetPos);
        runToPos = true;
        liftTimer.reset();
    }

    // Set lift position to middle
    public void setLiftMid() {
        targetPos = LIFT_MID_POS;
        liftMotor.setTargetPosition(targetPos);
        runToPos = true;
        liftTimer.reset();
    }

    // Set lift position to bottom
    public void setLiftBtm() {
        targetPos = LIFT_BTM_POS;
        liftMotor.setTargetPosition(targetPos);
        runToPos = true;
        liftTimer.reset();
    }

    // Move lift to updated lift position then stop
    public void updateLiftMotor() {

        double difference = Math.abs(liftMotor.getCurrentPosition() - targetPos);
        double liftPower = difference / 1000;
        Range.clip(liftPower, 0.2, 1.0);
        liftMotor.setPower(liftPower);
    }

    // Resets lift encoder to 0 using lift limit switch
    public void resetFloorPos() {
        if (liftLimit.getState()) {
            liftMotor.setPower(-0.3);
        } else {
            liftMotor.setPower(0.0);
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftTimer.reset();
        }
    }

    public boolean isResetComplete() {
        if (liftMotor.getMode() == DcMotor.RunMode.STOP_AND_RESET_ENCODER && liftTimer.milliseconds() > 1000) {
            return true;
        } else return false;
    }

    public boolean reachedFloor() {
        // return if reached our destination
        return true;
    }

}


