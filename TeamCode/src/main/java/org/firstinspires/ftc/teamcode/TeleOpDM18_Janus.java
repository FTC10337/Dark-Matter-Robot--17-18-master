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

@TeleOp(name="Teleop Janus", group="DM18")
//@Disabled
public class TeleOpDM18_Janus extends OpMode {

    /* Declare OpMode members. */
    HardwareDM18 robot = new HardwareDM18(); // use the class created to define a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.


    public enum States {INIT_1, INIT_2, INIT_3, RESET_1, RESET_2, RESET_2_1,  RESET_3, RESET_4, AUTO_LOAD_INIT, AUTO_LOAD_1, AUTO_LOAD_1_2, AUTO_LOAD_2, AUTO_LOAD_3, AUTO_LOAD_4, AUTO_LOAD_5, AUTO_LOAD_SECOND}

    States nStates = States.INIT_1;

    boolean init_TeleOp = true;
    boolean init_AutoLoad = false;
    boolean init_Reset = false;

    boolean autoMove = false;

    boolean resetLiftBtm = false;
    boolean resetLiftTop = false;
    boolean liftChangePos = false;

    boolean topGripisClosed = false;

    boolean isButtonPressed = false;

    boolean flip = false;

    boolean glyphDetected = false;

    int turnCoefficient = 1;

    int liftFloorTarget = 0;



    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, false);



        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");


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

        /*
        INITIATION SEQUENCE. RESETS LIFT AND GRIPPER
         */
        if (init_TeleOp) { // AUTO SEQUENCED STATES - INIT, LOAD & RESET
            switch (nStates) {
                case INIT_1: // INIT DRIVER CONTROL _ RESET LIFT TO TOP AND INIT GRIPPER
                    if (robot.lift.resetTopPos()) {
                        // initiates gripper. Init will set grippers closed, flipped in correct orientation, and pusher in home position.
                        robot.gripper.init(hardwareMap, "gripP", "gripB", "gripRotate", "gripExtend");
                        robot.gripper.setBothOpen();
                        robot.gripper.setExtendIn();
                        nStates = States.INIT_2;

                    }
                    break;

                case INIT_2: // Wait for the init moves to finish before we lower down

                    if (!robot.gripper.isFlipping()) {
                        nStates = States.INIT_3;
                        robot.lift.setLiftBtm();
                    }
                    break;

                case INIT_3: // INIT DRIVER CONTROL _ LIFT TO BOTTOM POSITION
                    telemetry.addData("Enc: ", robot.lift.liftMotor.getCurrentPosition());
                    if (robot.lift.reachedFloor()) {
                        if (robot.lift.resetFloorPos()) {
                            init_TeleOp = false;
                        }
                    }
                    break;
            }

        }

        /*
          DRIVE 1 CONTROLS
         */

        // Intake controls. Some intake controls do not function while auto load sequence is activate
        intakeControl();

        double left;
        double right;

        double throttle = -gamepad1.left_stick_y;
        double direction = gamepad1.right_stick_x;


        // Smooth and deadzone the joytick values
        throttle = smoothPowerCurve(deadzone(throttle, 0.10));

        // Change turn speed if trying to place glyph
        if (!init_TeleOp && robot.gripper.isPusherOut()) {
            turnCoefficient = 4;
        } else turnCoefficient = 2;

        direction = (smoothPowerCurve(deadzone(direction, 0.10))) / turnCoefficient;

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
          LIFT DRIVER_2 CONTROLS
         */

        // Determine if button is still pressed. Used to prevent multiple firing of robot actions when button is held down.
        if (!gamepad2.a && !gamepad2.x && !gamepad2.y && !gamepad2.b && !gamepad2.left_bumper && !gamepad2.right_bumper && !gamepad2.dpad_down
                && !gamepad2.dpad_left && !gamepad2.dpad_right && !gamepad2.dpad_up){
            isButtonPressed = false;
        }

        // Determine if glyph is detected in intake. Necessary to start auto-load sequence
        glyphDetected = robot.intake.detectGlyph();


        /*
        MANUAL LIFT CONTROLS - DRIVER 2
         */

        // / MANUAL MOVEMENT OF LIFT _ Moving right stick stops all auto movements
        if (gamepad2.right_stick_y > 0.2 && !init_TeleOp || gamepad2.right_stick_y < -0.2 && !init_TeleOp){

            // Override auto moves
            autoMove = false;
            init_AutoLoad = false;
            init_Reset = false;

            // reset all auto lift booleans to default
            resetLiftBtm = false;
            resetLiftTop = false;
            liftChangePos = false;
            flip = false;

            // Move lift
            robot.lift.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            double liftPower = -gamepad2.right_stick_y;
            liftPower = smoothPowerCurve(deadzone(liftPower, 0.20));
            if (!robot.lift.liftLimitT.getState()) {
                liftPower = Range.clip(liftPower, -1, 0);
            } else if (!robot.lift.liftLimitB.getState()){
                liftPower = Range.clip(liftPower, 0, 1);
            } else liftPower = Range.clip(liftPower, -1, 1);

            robot.lift.liftMotor.setPower(liftPower);

        } else if (!autoMove && !init_TeleOp) {
            // Stops lift if no joystick input or auto moves active
            robot.lift.stopLift();
        }

        // SETS LIFT POSITION
        if (gamepad2.dpad_up && !isButtonPressed) {
            isButtonPressed = true;
            liftFloorTarget = 2;            // Top floor
            liftChangePos = true;
            autoMove = true;
        }

        if ((gamepad2.dpad_left || gamepad2.dpad_right) && !isButtonPressed) {
            isButtonPressed = true;
            liftFloorTarget = 1;            // Middle floor
            liftChangePos = true;
            autoMove = true;
        }

        if (gamepad2.dpad_down && !isButtonPressed) {
            isButtonPressed = true;
            liftFloorTarget = 0;            // Bottom floor
            liftChangePos = true;
            autoMove = true;
        }


        // SETS LIFT POSITION AND MOVES LIFT
        if (liftChangePos && autoMove) {
            switch(liftFloorTarget){
                case 0:
                    robot.lift.setLiftBtm();
                    liftChangePos = false;
                    resetLiftBtm = true;
                    break;
                case 1:
                    robot.lift.setLiftMid();
                    liftChangePos = false;
                    break;
                case 2:
                    robot.lift.setLiftTop();
                    liftChangePos = false;
                    resetLiftTop = true;
                    break;
            }
        }

        // Find BTM limit switch after changing lift position BTM
        if (resetLiftBtm && robot.lift.reachedFloor() && autoMove) {
            if (robot.lift.resetFloorPos()){
                resetLiftBtm = false;
            }
        }

        // Find TOP limit switch after changing lift position to TOP
        if (resetLiftTop && robot.lift.reachedFloor() && autoMove) {
            if (robot.lift.resetTopPos()) {
                resetLiftTop = false;
            }
        }

        /*
          GRIPPER CONTROLS - DRIVER 2
         */

        // INITIATE FLIP
        if (gamepad2.x && !isButtonPressed && !init_TeleOp) {
            isButtonPressed = true;
            flip = true;
            autoMove = true;
        }

        // FLIP GRIPPER
        if (flip && autoMove && (robot.lift.distFromBottom() >= 9.0)) {
            // Flip after determining gripper is high enough
            robot.gripper.flip();
            flip = false;
        } else if (flip && autoMove && robot.lift.distFromBottom() < 9.0) {
            // Move gripper to top position before flipping if in another lift position
            robot.lift.setLiftHeight(9.5);
        }

        // PUSHER IN/OUT
        if (gamepad2.left_stick_y > 0.2 && !init_TeleOp || gamepad2.left_stick_y < -0.2 && !init_TeleOp) {
            telemetry.addData("Push In/Out DFB ", robot.lift.distFromBottom());
            telemetry.addData("Intake open: ", !robot.intake.isClosed());
            telemetry.addData("Intake moving: ", robot.intake.isMoving());
            telemetry.addData("Grip Btm Closed: ", robot.gripper.isBtmClosed());
            telemetry.addData("Grip Btm Partial: ", robot.gripper.isBtmPartialOpen());
            telemetry.addData("Grip Btm Moving: ", robot.gripper.isMoving());

            // Override auto moves
            autoMove = false;
            init_AutoLoad = false;
            init_Reset = false;

            // reset all auto lift booleans to default
            resetLiftBtm = false;
            resetLiftTop = false;
            liftChangePos = false;
            flip = false;


            if ((robot.lift.distFromBottom() > 8) ||
                    (!robot.intake.isClosed() && !robot.intake.isMoving() &&
                            (robot.gripper.isBtmClosed() || robot.gripper.isBtmPartialOpen()) && !robot.gripper.isMoving())) {
                double speed = smoothPowerCurve(deadzone(-gamepad2.left_stick_y, 0.2));
                speed = Range.clip(speed, -1, 1);
                robot.gripper.moveInOut(speed);
            } else {
                robot.intake.setOpen();
                if (robot.gripper.isBtmOpen()) {
                    robot.gripper.setBtmPartialOpen();

                }
            }
        }


        // OPEN & CLOSE GRIPPERS
        // close grippers
        if ((gamepad2.right_trigger > 0.5) && !init_TeleOp) {
            robot.gripper.setBtmClosed();
        }
        if ((gamepad2.right_bumper && !isButtonPressed && !init_TeleOp)) {
            robot.gripper.setTopClosed();
            isButtonPressed = true;
            topGripisClosed = true;
        }

        // open grippers partially if pusher is extended or fully if not
        if ((gamepad2.left_trigger > 0.5) && !init_TeleOp) {
            if (robot.gripper.isPusherOut()) {
                // Set to partial open since we are extended
                robot.gripper.setBtmPartialOpen();
            } else {
                // Set fully open since we are all the way in
                robot.gripper.setBtmOpen();
            }
        }
        if ((gamepad2.left_bumper && !isButtonPressed && !init_TeleOp)) {
            if (robot.gripper.isPusherOut()) {
                // Set to partial open since we are extended
                robot.gripper.setTopPartialOpen();
                topGripisClosed = true;
                isButtonPressed = true;
            } else {
                // Set fully open since we are all the way in
                robot.gripper.setTopOpen();
                topGripisClosed = false;
                isButtonPressed = true;
            }
        }

        /*
        AUTO RESET LIFT/GRIPPER SEQUENCE
         */

        // RESET auto load sequence. To be used by both drivers is something in sequence goes wrong
        if (gamepad2.y && gamepad2.b && !isButtonPressed && !init_TeleOp) {
            isButtonPressed = true;
            autoMove = true;
            init_Reset = true;
            init_AutoLoad = false;
            nStates = States.RESET_1;
        }

        if (init_Reset && autoMove) {
            switch (nStates) {

                case RESET_1: // OPEN INTAKE
                    telemetry.clearAll();
                    telemetry.addData("Reset: ", "1");
                    if (robot.intake.isClosed()) {
                        robot.intake.setOpen();
                        nStates = States.RESET_2;
                    }
                    break;

                case RESET_2: // LIFT ABOVE INTAKE IF NOT ALREADY
                    telemetry.clearAll();
                    telemetry.addData("Reset: ", "2");
                    if (!robot.intake.isMoving()){
                        if (robot.lift.distFromBottom() < 8.5) {
                            robot.lift.setLiftHeight(8.5);
                            nStates = States.RESET_2_1;
                        } else nStates = States.RESET_2_1;
                    }

                case RESET_2_1: // PUSHER IN & OPEN GRIPPERS
                    telemetry.clearAll();
                    telemetry.addData("Reset: ", "2_1");
                    if (robot.lift.reachedFloor()) {
                        robot.gripper.setExtendIn();
                        robot.gripper.setBothOpen();
                        topGripisClosed = false;
                        nStates = States.RESET_3;
                    }
                    break;

                case RESET_3:
                    telemetry.clearAll();
                    telemetry.addData("Reset: ", "3");
                    if (!robot.gripper.isExtending()) {
                        robot.lift.setLiftBtm();
                        nStates = States.RESET_4;
                    }
                    break;

                case RESET_4:
                    telemetry.clearAll();
                    telemetry.addData("Reset: ", "4");
                    if (robot.lift.reachedFloor()){
                        if (robot.lift.resetFloorPos()) {
                            robot.intake.setClosed();
                            robot.intake.setIn();
                            init_Reset = false;
                        }
                    }

            }
        }


        /*
        AUTO LOAD GLYPH SEQUENCE
         */

        // INITIATE AUTO LOAD - DRIVER 2 - Can be activated when glyph is detected
        if (gamepad2.a && glyphDetected && !isButtonPressed && !init_TeleOp) {
            isButtonPressed = true;
            nStates = States.AUTO_LOAD_INIT;
            init_AutoLoad = true;
            autoMove = true;
            init_Reset = false;
        }

        if (init_AutoLoad && autoMove) {

            switch (nStates) {

                case AUTO_LOAD_INIT: // CHECK TO SEE IF LIFT IS AT BTM POS AND BTM GRIPPERS ARE OPEN
                    if (robot.gripper.isBtmClosed()) {
                        robot.gripper.setBtmOpen();
                    } else if (!robot.gripper.btmIsMoving()){
                        robot.lift.setLiftBtm();
                    }
                    if (!robot.gripper.isBtmClosed()&& robot.lift.targetPos == robot.lift.LIFT_BTM_POS && robot.lift.reachedFloor()){
                        // Move lift down to BTM limit switch
                        if (robot.lift.resetFloorPos()) {
                            nStates = States.AUTO_LOAD_1;
                        }
                    }
                    break;

                case AUTO_LOAD_1: // GRAB GLYPH & OPEN INTAKE WHEELS
                    // Grab glyph with BTM gripper
                    if(!robot.gripper.isBtmClosed()) {
                        robot.gripper.setBtmClosed();
                        nStates = States.AUTO_LOAD_1_2;
                    }
                    break;

                case AUTO_LOAD_1_2:
                    // Open intake after gripper has closed
                    telemetry.clearAll();
                    telemetry.addData("Top is closed:", topGripisClosed);
                    if (!robot.gripper.btmIsMoving()) {
                        robot.intake.setOpen();
                        if (!topGripisClosed) {
                            nStates = States.AUTO_LOAD_2;
                        } else {
                            nStates = States.AUTO_LOAD_SECOND;
                        }
                    }
                    break;

                // LOAD SEQUENCE FOR FIRST GLYPH - Determined if top gripper is OPEN, therefore does not have glyph.
                case AUTO_LOAD_2: // LIFT TO TOP
                    // Set lift position to move to top after intake has opened
                    if (!robot.intake.isMoving()) {
                        robot.lift.setLiftTop();
                        nStates = States.AUTO_LOAD_3;
                    }
                    break;

                case AUTO_LOAD_3: // FLIP AFTER TOP REACHED
                    if (robot.lift.distFromBottom() > 9.0) { // Check to see if lift reached height to flip
                        // If it reached target position, set intake back to closed position and start intaking again
                        robot.intake.setClosed();
                        robot.intake.setIn();
                        robot.gripper.flip(); // Flip gripper
                        topGripisClosed = true;
                        nStates = States.AUTO_LOAD_4; // Go to next state in auto load sequence
                    }
                    break;

                case AUTO_LOAD_4: // MOVE TO BTM FLOOR

                    if (!robot.gripper.isFlipping()) {
                        robot.lift.setLiftHeight(6.0); // Move lift to btm position after gripper is done flipping
                        //nStates = States.AUTO_LOAD_5;
                        init_AutoLoad = false;
                    }
                    break;

                case AUTO_LOAD_5: // RESET ENCODER TO '0' USING BTM LIMIT SWITCH
                    if (robot.lift.reachedFloor()) {
                        robot.lift.resetFloorPos();
                        init_AutoLoad = false;
                    }
                    break;

                // LOAD SEQUENCE FOR SECOND GLYPH - Determined if top gripper is CLOSED, therefore assumed it has a glyph.
                case AUTO_LOAD_SECOND: // LIFT TO DRIVE
                    if (!robot.intake.isMoving()) {
                        robot.lift.setLiftHeight(2.0);
                        init_AutoLoad = false;
                    }
                    break;
            }
        }

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
        if (gamepad1.right_trigger > 0.5 && !init_AutoLoad && !init_Reset) {
            robot.intake.setIn();
            robot.intake.setClosed();
        }

        // Intake STOP
        if (robot.intake.isIntakeInOn && robot.intake.detectGlyph()) {
            robot.intake.setStop();
        }

        // Intake OUT
        if (gamepad1.left_trigger > 0.5 && !init_AutoLoad && !init_Reset) {
            robot.intake.setOut();
            robot.intake.setClosed();
        }

        // Intake STOP
        if (gamepad1.a && !init_AutoLoad && !init_Reset) {
            robot.intake.setStop();
        }
        // Intake open & close
        if (gamepad1.x && !init_AutoLoad && !init_Reset) {
            robot.intake.setClosed();
        }
        if (gamepad1.b && !init_AutoLoad && !init_Reset) {
            robot.intake.setOpen();

        }

    }



}

