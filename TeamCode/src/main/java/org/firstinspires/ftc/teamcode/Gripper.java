package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


/**
 *    Everything related to the gripper
 */
public class Gripper {

    // Hardware
    public Servo purpleGrip = null;
    public Servo blackGrip = null;
    public Servo rotateServo = null;

    // Servo constants
    public final static double GRIP_OPEN = 0.0;
    public final static double GRIP_CLOSED = 1.0;
    public final static double GRIP_ROTATE_NORMAL = 0.0;
    public final static double GRIP_ROTATE_FLIPPED = 1.0;

    /* Gripper state variables */
    Servo topGrip = purpleGrip;        // Should start w/ purple gripper on top
    Servo btmGrip = blackGrip;         // and black on bottom
    boolean isGripFlipped = false;


    /**
     * Constructor
     */
    public void Gripper() {
        // Do nothing
    }

    /**
     * Initialize the gripper
     *
     * @param hw  Hardwaremap for our robot
     * @param pg  Name of purple gripper servo
     * @param bg  Name of black gripper servo
     * @param rot Name of gripper rotation servo
     */
    public void init(HardwareMap hw, String pg, String bg, String rot) {
        // Define and Initialize gripper servos
        purpleGrip = hw.servo.get(pg);
        blackGrip = hw.servo.get(bg);
        rotateServo = hw.servo.get(rot);

        // Start with purple on top
        topGrip = purpleGrip;
        btmGrip = blackGrip;
        isGripFlipped = false;

        setFlipped(false);
        setBothClosed();
    }

    /**
     * Open the purple gripper
     */
    public void setPurpleOpen() {
        purpleGrip.setPosition(GRIP_OPEN);
    }

    /**
     * Open the black gripper
     */
    public void setBlackOpen() {
        blackGrip.setPosition(GRIP_OPEN);
    }

    /**
     * Open whichever gripper is currently on top
     */
    public void setTopOpen() {
        topGrip.setPosition(GRIP_OPEN);
    }

    /**
     * Open whichever gripper is currently on bottom
     */
    public void setBtmOpen() {
        btmGrip.setPosition(GRIP_OPEN);
    }

    /**
     * Open both grippers
     */
    public void setBothOpen() {
        setTopOpen();
        setBtmOpen();
    }


    /**
     * Close the purple gripper
     */
    public void setPurpleClosed() {
        purpleGrip.setPosition(GRIP_CLOSED);
    }

    /**
     * Close the black gripper
     */
    public void setBlackClosed() {
        blackGrip.setPosition(GRIP_CLOSED);
    }

    /**
     * Close whichever gripper is currently on top
     */
    public void setTopClosed() {
        topGrip.setPosition(GRIP_CLOSED);
    }

    /**
     * Close whichever gripper is currently on bottom
     */
    public void setBtmClosed() {
        btmGrip.setPosition(GRIP_CLOSED);
    }

    /**
     * Close both grippers
     */
    public void setBothClosed() {
        setTopClosed();
        setBtmClosed();
    }


    public boolean isTopClosed() {
        return (topGrip.getPosition() == GRIP_CLOSED);
    }

    public boolean isBtmClosed() {
        return (btmGrip.getPosition() == GRIP_CLOSED);
    }

    public boolean isPurpleClosed() {
        return (purpleGrip.getPosition() == GRIP_CLOSED);
    }

    public boolean isBlackClosed() {
        return (blackGrip.getPosition() == GRIP_CLOSED);
    }

    public void flip() {
        if (isGripFlipped) {
            // Was flipped so turn it back upright
            setFlipped(false);
        } else {
            // Was not flipped so turn it upside down
            setFlipped(true);
        }
    }

    public void setFlipped(boolean flipped) {
        if (flipped) {
            rotateServo.setPosition(GRIP_ROTATE_FLIPPED);
            isGripFlipped = true;
            topGrip = blackGrip;
            btmGrip = purpleGrip;
        }
        else {
            rotateServo.setPosition(GRIP_ROTATE_NORMAL);
            isGripFlipped = false;
            topGrip = purpleGrip;
            btmGrip = blackGrip;
        }
    }

    /**
     * This is designed to be used for manual tuning of servo Min/Max constants not for routine use
     *
     * @return current top grip servo position
     */
    public double getTopServoPos() {
        return topGrip.getPosition();
    }

    /**
     * This is designed to be used for manual tuning of servo Min/Max constants not for routine use
     *
     * @return current btm grip servo position
     */

    public double getBtmServoPos() {
        return btmGrip.getPosition();
    }

    /**
     * This is designed to be used for manual tuning of servo Min/Max constants not for routine use
     *
     * @param pos desired top grip servo position
     */
    public void setTopServoPos(double pos) {
        pos = Range.clip(pos, 0.0, 1.0);
        topGrip.setPosition(pos);
    }

    /**
     * This is designed to be used for manual tuning of servo Min/Max constants not for routine use
     *
     * @param pos desired bottom grip servo position
     */
    public void setBtmServoPos(double pos) {
        pos = Range.clip(pos, 0.0, 1.0);
        btmGrip.setPosition(pos);
    }


    /**
     * This is designed to be used for manual tuning of servo Min/Max constants not for routine use
     *
     * @return current rotation servo position
     */
    public double getRotServoPos() {
        return rotateServo.getPosition();
    }

    /**
     * This is designed to be used for manual tuning of servo Min/Max constants not for routine use
     *
     * @param pos desired rotate servo position
     */
    public void setRotServoPos(double pos) {
        pos = Range.clip(pos, 0.0, 1.0);
        rotateServo.setPosition(pos);
    }

}

