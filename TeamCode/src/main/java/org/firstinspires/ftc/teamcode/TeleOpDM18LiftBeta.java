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

@TeleOp(name="Teleop Tank Beta", group="DM18")
@Disabled
public class TeleOpDM18LiftBeta extends OpMode {

    /* Declare OpMode members. */
    HardwareDM18 robot = new HardwareDM18(); // use the class created to define a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.


    boolean intake = false;

    int curState = 0;       // Not in an automated sequence
    int lastState = curState;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, true);

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


        // Process the intake and gripper state model

        // First we need to give intake a chance to cycle powers each cycle if needed
        robot.intake.updateInPower();


        switch (curState) {
            case 0:  {   // Not currently intaking or placing
                if (gamepad2.a) {
                    // Go to intaking glyphs mode
                    curState = 1;
                } else if (gamepad2.b) {
                    // Go to placing glyph mode
                    curState = 101;
                }
                break;
            }
            case 1: {  // Ready to intake
                if (curState != lastState) {
                    robot.gripper.setBothOpen();
                    robot.intake.setClosed();
                    robot.intake.setIn();
                }
            }
        }


        lastState = curState;

       /*
        // Intake IN
        if (gamepad1.right_trigger > 0.5) {
            intake = true;
            }


        if (intake == true) {
            robot.intakeLeftMotor.setPower(1.0);
            robot.intakeRightMotor.setPower(0.6);
            }

        if (intake == true && robot.jewelDS.getDistance(DistanceUnit.CM) < 7.0) {
            robot.intakeLeftMotor.setPower(0.0);
            robot.intakeRightMotor.setPower(0.0);
            intake = false;
            }

        // Intake OUT

        if (gamepad1.left_trigger > 0.5) {
            robot.intakeLeftMotor.setPower(-1.0);
            robot.intakeRightMotor.setPower(-1.0);
            intake = false;
            }

        // Intake STOP
        if (gamepad1.a) {
            robot.intakeLeftMotor.setPower(0.0);
            robot.intakeRightMotor.setPower(0.0);
            intake = false;
            }


        // Intake open & close
        if (gamepad1.x) {
            robot.intakeLeftServo.setPosition(robot.INTAKE_LEFT_HOME);
            robot.intakeRightServo.setPosition(robot.INTAKE_RIGHT_HOME);
            }
        if (gamepad1.b) {
            robot.intakeLeftServo.setPosition(robot.INTAKE_LEFT_RELEASE);
            robot.intakeRightServo.setPosition(robot.INTAKE_RIGHT_RELEASE);

         }
    */

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