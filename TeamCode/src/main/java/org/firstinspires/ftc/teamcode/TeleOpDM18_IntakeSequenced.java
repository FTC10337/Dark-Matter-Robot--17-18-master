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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static java.lang.Double.NaN;

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

@TeleOp(name="Teleop Auto_Load", group="DM18")
//@Disabled
public class TeleOpDM18_IntakeSequenced extends OpMode {

    /* Declare OpMode members. */
    HardwareDM18 robot = new HardwareDM18(); // use the class created to define a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.


    boolean intake = false;
    int curState = 0;
    int lastState = 0;

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


        telemetry.addData("curState: ", curState);
        telemetry.addData("Enc: ", robot.lift.liftMotor.getCurrentPosition());
        telemetry.addData("lPower", robot.intake.lInPower);
        telemetry.update();

        double left;
        double right;

        double throttle = -gamepad1.left_stick_y;
        double direction = gamepad1.right_stick_x;

        // Smooth and deadzone the joytick values
        throttle = smoothPowerCurve(deadzone(throttle, 0.10));
        direction = (smoothPowerCurve(deadzone(direction, 0.10))) / 2;

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



        switch (curState) {
            case 0: // BEGINNING OF TELEOP
                if (curState != lastState) robot.intake.setClosed();
                // If driver_1 starts intake, move on to next state
                if (gamepad1.right_trigger > 0.2) {
                    robot.intake.setIn();
                    curState = 1;
                } else if (gamepad1.left_trigger > 0.2) {
                    robot.intake.setOut();
                } else if (gamepad1.a) {
                    robot.intake.setStop();
                }
                break;

            case 1: // ATTEMPTING TO INTAKE GLYPH
                // If glyph is detected, stop intake and move on to next state
                if (robot.jewelDS.getDistance(DistanceUnit.CM) < 7.0) {
                    robot.intake.setStop();
                    curState = 2;
                } else if (gamepad1.left_trigger > 0.2) {
                    robot.intake.setOut();
                    curState = 0;
                } else if (gamepad1.a) {
                    robot.intake.setStop();
                    curState = 0;
                }
                break;

            case 2: // GLYPH DETECTED IN INTAKE
                // Driver has ability to push glyph out to get another color if needed and reset process
                if (gamepad1.left_trigger > 0.2) {
                    robot.intake.setOut();
                    curState = 0;
                }
                // Initiate grab of glyph and move on to next state if driver_2 confirms.
                // ***AUTO SEQUENCE BEGINS AFTER THIS DRIVER CONFIRMATION***
                if (gamepad2.a && robot.jewelDS.getDistance(DistanceUnit.CM) < 7.0) {
                    robot.gripper.setBtmClosed();
                    curState = 3;
                }
                // If for some reason, glyph is not seen, go back to start of process.
                if (robot.jewelDS.getDistance(DistanceUnit.CM) > 7.0) curState = 0;
                break;

            case 3: // OPEN INTAKE WHEELS
                // Open intake after gripper has closed
                if (!robot.gripper.isMoving()) {
                    robot.intake.setOpen();
                    curState = 4;
                }
                break;

            case 4: // SET LIFT TO TOP
                // Set lift position to move to top after intake has opened
                if (!robot.intake.isMoving()) {
                    robot.lift.setLiftTop();
                    curState = 5;
                }
                break;

            case 5: // LIFT TO TOP
                // Move lift to top position
                if (robot.lift.runToPos) {
                    robot.lift.updateLiftMotor();
                } else {
                    // When lift makes it to top, set intake back to closed position and start intaking again
                    // Flip gripper
                    robot.intake.setClosed();
                    robot.intake.setIn();
                    robot.gripper.flip();
                    curState = 6;
                }
                break;

            case 6: // MOVE TO BTM FLOOR AND START LOOKING FOR ANOTHER GLYPH
                // Set lift to btm position
                if (curState != lastState)  robot.lift.setLiftBtm();
                // Begin moving lift to btm position after gripper is done flipping
                if (!robot.gripper.isFlipping()) {
                    robot.lift.updateLiftMotor();
                }
                // Stop intake after glyph has been detected
                if (robot.jewelDS.getDistance(DistanceUnit.CM) < 7.0) {
                    robot.intake.setStop();
                }
                // Move to next state when flipping is done and glyph has been detected in intake
                if (!robot.gripper.isFlipping() && robot.jewelDS.getDistance(DistanceUnit.CM) < 7.0) {
                    curState = 7;
                }
                break;

            case 7: // READY TO GRAB 2ND GLYPH AS SOON AS LIFT REACHES BOTTOM
                // Continue to lower lift
                if (robot.lift.runToPos) {
                    robot.lift.updateLiftMotor();
                } else
                // Lift in down position. New options available
                {
                    // set intake in
                    if (gamepad1.right_trigger > 0.2) {
                        robot.intake.setIn();
                    } // set intake out
                    else if (gamepad1.left_trigger > 0.2) {
                        robot.intake.setOut();
                    } // set intake stop
                    else if (gamepad1.a) {
                        robot.intake.setStop();
                    } // set intake stop if glyph detected
                    else if (robot.jewelDS.getDistance(DistanceUnit.CM) < 7.0) {
                        robot.intake.setStop();
                    }

                    // Move to next state if driver2 initiates glyph grab
                    if (gamepad2.a && !robot.intake.isIntakeInOn && !robot.intake.isIntakeOutOn && robot.jewelDS.getDistance(DistanceUnit.CM) < 7.0) {
                        // Grab glyph
                        robot.gripper.setBtmClosed();
                        curState = 8;
                    }
                }
                break;

            case 8: // OPEN INTAKE WHEELS
                if (!robot.gripper.isMoving()) {
                    robot.intake.setOpen();
                    curState = 9;
                }
                break;

            case 9: // LIFT FOR DRIVING TO SCORE GLYPHS
                if (!robot.intake.isMoving()) {
                    robot.lift.setLiftMid();
                    curState = 10;
                }
                break;

            case 10: // CONTINUE TO LIFT. TRANSITION TO SCORING STATE
                robot.lift.updateLiftMotor();
                if (!robot.lift.runToPos) {
                    curState = 101;
                }
                break;

            case 101: // GLYPHS LOADED. DRIVING TO SCORE STATE INITITATED
                if (curState != lastState) robot.intake.setOpen();

                // Flip glyph
                if (gamepad2.x && robot.lift.targetPos == robot.lift.LIFT_TOP_POS) curState = 102;

                    // Lift to btm floor
                else if (gamepad2.a) curState = 110;

                    // Lift to mid floor
                else if (gamepad2.b) curState = 111;

                    // Lift to top floor
                else if (gamepad2.y) curState = 112;

                    // Extend gripper out
                else if (gamepad2.right_trigger > 0.2) curState = 120;

                break;


            case 102: // FLIP SEQUENCE
                if (curState != lastState) robot.gripper.flip();
                if (!robot.gripper.isFlipping()) curState = 101;
                break;

            case 110: // MOVE TO BTM FLOOR
                if (curState != lastState) robot.lift.setLiftBtm();
                if (robot.lift.runToPos) robot.lift.updateLiftMotor();
                else curState = 101;
                break;

            case 111: // MOVE TO MID FLOOR
                if (curState != lastState) robot.lift.setLiftMid();
                if (robot.lift.runToPos) robot.lift.updateLiftMotor();
                else curState = 101;
                break;

            case 112: // MOVE TO TOP FLOOR
                if (curState != lastState) robot.lift.setLiftTop();
                if (robot.lift.runToPos) robot.lift.updateLiftMotor();
                else curState = 101;
                break;

            case 120: // EXTEND PUSHER
                if (!robot.intake.isClosed()) {
                    robot.gripper.setExtendOut();
                    curState = 130;
                }
                break;

            case 130: // PLACE GLYPH
                // retrieve pusher
                if (gamepad2.left_trigger > 0.2) {
                    robot.gripper.setExtendIn();
                    curState = 101;
                }
                // partially open grippers to drop glyphs without interfering with other stacked glyphs
                if (gamepad2.right_bumper && !robot.gripper.isExtending()) robot.gripper.setBothPartialOpen();

                // close grippers
                if (gamepad2.left_bumper) robot.gripper.setBothClosed();

                // RESET intake system
                if (gamepad2.dpad_down) {
                    robot.gripper.setBothClosed();
                    curState = 777;
                }
                break;

            case 777: // RESET FOR AUTO LOAD SEQUENCE
                if (!robot.gripper.isMoving()) {
                    // reset lift position to btm
                    robot.lift.setLiftBtm();
                    // reset gripper to in position
                    robot.gripper.setExtendIn();

                    if(robot.lift.runToPos) {
                        // move lift
                        robot.lift.updateLiftMotor();
                    } else if (robot.gripper.isExtending()) {
                        // Return to driving state
                        curState = 1000;
                    }

                }
                break;

            case 1000: // RESET FLOOR POSITION AND RESET ENCODERS
                if (robot.lift.liftMotor.getMode() != DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
                    robot.lift.reachedFloor();
                } else {
                    curState = 1001;
                }
                break;

            case 1001: // TRANSITION BACK TO DRIVER CONTROL AFTER LIFT ENCODER HAS HAD TIME TO RESET
                if (robot.lift.isResetComplete()) {
                    robot.lift.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    curState = 0;
                }
                break;

            default:
                intakeControl();
                break;
        }

        lastState = curState;

        // Give the intake a chance to adjust speeds in cycle
        robot.intake.updateInPower();   // Must be called each cycle for speed to vary properly

    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop () {
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
        double b = 0.05;      // Min power to overcome motor stall

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
     * @param dz   Deadzone value to use 0 to 0.999
     * @return    Value after deadzone processing
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


// Intake Control

    public void intakeControl()
    {


        // Intake IN
        if (gamepad1.right_trigger > 0.5) {
            robot.intake.setIn();
            intake = true;
        }

        // Intake STOP
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
        // Intake open & close
        if (gamepad1.x) {
            robot.intake.setClosed();
        }
        if (gamepad1.b) {
            robot.intake.setOpen();

        }

    }


}
