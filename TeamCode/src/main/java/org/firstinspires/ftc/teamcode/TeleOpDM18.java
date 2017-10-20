/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Teleop Tank", group="DM18")
//@Disabled
public class TeleOpDM18 extends OpMode {

    /* Declare OpMode members. */
    HardwareDM18 robot = new HardwareDM18(); // use the class created to define a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.


    boolean intake = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        // Telemetry statements for displaying & tuning servos
        double jewel_pos = robot.jewelServo.getPosition();
        double jewel_rot_pos = robot.jewelRotServo.getPosition();
        double grip_top_pos = robot.gripper.getTopServoPos();
        double grip_bottom_pos = robot.gripper.getBtmServoPos();
        double intake_left_pos = robot.intake.getLeftServoPos();
        double intake_right_pos = robot.intake.getRightServoPos();
        //telemetry.addData("Dist: ", robot.jewelDS.getDistance(DistanceUnit.CM));
        //telemetry.addData("left Pos: ", intake_left_pos);
        //telemetry.addData("right Pos: ", intake_right_pos);
        //telemetry.addData("top Grip Pos:", grip_top_pos);
        //telemetry.addData("btm Grip Pos:", grip_bottom_pos);

        telemetry.update();

        double left;
        double right;

        double throttle = -gamepad1.left_stick_y;
        double direction = gamepad1.right_stick_x;

        // Smooth and deadzone the joytick values
        throttle = smoothPowerCurve(deadzone(throttle, 0.10));
        direction = (smoothPowerCurve(deadzone(direction, 0.10)))/2;

        // Calculate the drive motors for left and right
        right = throttle - direction;
        left = throttle + direction;

        // clip the right/left values so that the values never exceed +/- 1
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);
        // Normalize speeds if any one exceeds +/- 1.0;
        double max = Math.max(Math.abs(right), Math.abs(left));
        if (max > 1.0) {
            left /= max;
            right /= max;
        }

        robot.leftDrive1.setPower(left);
        robot.leftDrive2.setPower(left);
        robot.rightDrive1.setPower(right);
        robot.rightDrive2.setPower(right);


        /*
        // Block of code used to calibrate the intake servo position limits.
        // Normally commented out

        if (gamepad1.left_bumper) intake_left_pos+=0.01;
        if (gamepad1.right_bumper) intake_left_pos-=0.01;

        intake_left_pos = Range.clip(intake_left_pos, 0.0, 1.0);
        robot.intake.setLeftServoPos(intake_left_pos);

        if (gamepad2.left_bumper) intake_right_pos+=0.01;
        if (gamepad2.right_bumper) intake_right_pos-=0.01;

        intake_right_pos= Range.clip(intake_right_pos, 0.0, 1.0);
        robot.intake.setRightServoPos(intake_right_pos);
        */


        /*  Block of code used to calibrate jewel arm
        if (gamepad1.y) jewel_pos+=0.001;

        if (gamepad1.a) jewel_pos-=0.001;

        jewel_pos = Range.clip(jewel_pos, robot.JEWEL_HOME, robot.JEWEL_DEPLOY);
        robot.jewelServo.setPosition(jewel_pos);

        if (gamepad2.y) jewel_rot_pos += 0.001;
        if (gamepad2.a) jewel_rot_pos -= 0.001;
        jewel_rot_pos = Range.clip(jewel_rot_pos, 0, 1.0);
        robot.jewelRotServo.setPosition(jewel_rot_pos);
        */

        /*  Block of code used to calibrate gripper servos
        if (gamepad1.left_bumper) grip_top_pos+=0.01;
        if (gamepad1.right_bumper) grip_top_pos-=0.01;

        grip_top_pos = Range.clip(grip_top_pos, 0.0, 1.0);
        robot.gripper.setTopServoPos(grip_top_pos);

        if (gamepad2.left_bumper) grip_bottom_pos+=0.01;
        if (gamepad2.right_bumper) grip_bottom_pos-=0.01;

        grip_bottom_pos = Range.clip(grip_bottom_pos, 0.0, 1.0);
        robot.gripper.setBtmServoPos(grip_bottom_pos);
        */


        // Process the grip flipper
        if (gamepad1.dpad_left) robot.gripper.flip();
        if (gamepad1.dpad_up) robot.gripper.setFlipped(false);
        if (gamepad1.dpad_up) robot.gripper.setFlipped(true);


        // Intake IN
        if (gamepad1.right_trigger > 0.5) {
            intake = true;
            robot.intake.setIn();
            }


        if (intake == true && robot.jewelDS.getDistance(DistanceUnit.CM) < 7.0) {
            robot.intake.setStop();
            intake = false;
            }

        // Intake OUT

        if (gamepad1.left_trigger > 0.5) {
            robot.intake.setOut();
            intake = false;
            }

        // Intake STOP
        if (gamepad1.a) {
            robot.intake.setStop();
            intake = false;
            }


        // Give the intake a chance to adjust speeds in cycle
        robot.intake.updateInPower();   // Must be called each cycle for speed to vary properly


        // Intake open & close
        if (gamepad1.x) {
            robot.intake.setClosed();
            }
        if (gamepad1.b) {
            robot.intake.setOpen();

            }


    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
        @Override
        public void stop() {
        }


    /**
     * This does the cubic smoothing equation on joystick value.
     * Assumes you have already done any deadzone processing.
     *
     * @param x  joystick input
     * @return  smoothed value
     */
    protected double smoothPowerCurve (double x) {
        //double a = this.getThrottle();
        double a = 1.0;         // Hard code to max smoothing
        double b = 0.05;		// Min power to overcome motor stall

        if (x > 0.0)
            return (b + (1.0-b)*(a*x*x*x+(1.0-a)*x));

        else if (x<0.0)
            return (-b + (1.0-b)*(a*x*x*x+(1.0-a)*x));
        else return 0.0;
    }

    /**
     * Add deadzone to a stick value
     *
     * @param rawStick  Raw value from joystick read -1.0 to 1.0
     * @param dz	Deadzone value to use 0 to 0.999
     * @return		Value after deadzone processing
     */
    protected double deadzone(double rawStick, double dz) {
        double stick;

        // Force limit to -1.0 to 1.0
        if (rawStick > 1.0) {
            stick = 1.0;
        } else if (rawStick < -1.0) {
            stick = -1.0;
        } else {
            stick = rawStick;
        }

        // Check if value is inside the dead zone
        if (stick >= 0.0){
            if (Math.abs(stick) >= dz)
                return (stick - dz)/(1 -  dz);
            else
                return 0.0;

        }
        else {
            if (Math.abs(stick) >= dz)
                return (stick + dz)/(1 - dz);
            else
                return 0.0;

        }
    }


}