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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.Range;

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

@TeleOp(name="Mechanism Manipulator", group="DM18")
//@Disabled
public class TeleOpDM18_MechManipulator extends OpMode {

    /* Declare OpMode members. */
    HardwareDM18 robot = new HardwareDM18(); // use the class created to define a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.


    boolean intake = false;
    boolean resetLift = false;
    boolean resetLiftTop = false;
    boolean autoLift = false;
    boolean plusCurState = false;
    boolean minusCurState = false;
    boolean isButtonPressed = false;
    double intakeDistance = 8.0;
    int curState = 0;
    int lastState = 0;
    double Pos = 0;
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
        robot.gripper.setBothOpen();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {


        telemetry.update();


        // Ability to change states. Each state gives controls to different robot mechanisms
        if (gamepad2.dpad_right && !isButtonPressed) {
            curState += 1;
            isButtonPressed = true;
            telemetry.clearAll();
        } else if (gamepad2.dpad_left && !isButtonPressed) {
            curState -= 1;
            isButtonPressed = true;
            telemetry.clearAll();
        }
        curState%=13;   // Wrap around

        if (!gamepad2.dpad_left && !gamepad2.dpad_right && isButtonPressed)isButtonPressed = false;




        switch (curState) {
            case 0: // GRIPPER BOTTOM
                telemetry.addData("GRIPPER", "PURPLE");
                telemetry.addData("Pos: ", robot.gripper.purpleGrip.getPosition());

                Pos = robot.gripper.purpleGrip.getPosition();

                if (gamepad2.x) {
                    Pos += 0.001;
                } else if (gamepad2.b) {
                    Pos -= 0.001;
                }

                Range.clip(Pos, 0, 1);
                robot.gripper.purpleGrip.setPosition(Pos);
                break;

            case 1: // GRIPPER TOP
                telemetry.addData("GRIPPER", "BLACK");
                telemetry.addData("Pos: ", robot.gripper.blackGrip.getPosition());

                Pos = robot.gripper.blackGrip.getPosition();

                if (gamepad2.x) {
                    Pos += 0.001;
                } else if (gamepad2.b) {
                    Pos -= 0.001;
                }

                Range.clip(Pos, 0, 1);
                robot.gripper.blackGrip.setPosition(Pos);
                break;

            case 2: // GRIPPER FLIP
                telemetry.addData("GRIPPER", "FLIP");
                telemetry.addData("Pos: ", robot.gripper.getRotServoPos());

                Pos = robot.gripper.getRotServoPos();

                if (gamepad2.x) {
                    Pos += 0.001;
                    Range.clip(Pos, 0, 1);
                    robot.gripper.setRotServoPos(Pos);
                } else if (gamepad2.b) {
                    Pos -= 0.001;
                    Range.clip(Pos, 0, 1);
                    robot.gripper.setRotServoPos(Pos);
                }

                if (gamepad2.right_bumper && !robot.gripper.isFlipping()) robot.gripper.flip();

                break;

            case 3: // GRIPPER PUSHER
                telemetry.addData("GRIPPER", "PUSHER");
                telemetry.addData("Pos: ", robot.gripper.extendGrip.getPosition());

                Pos = robot.gripper.extendGrip.getPosition();

                if (gamepad2.x) {
                    Pos += 0.001;
                } else if (gamepad2.b) {
                    Pos -= 0.001;
                }

                Range.clip(Pos, 0, 1);
                robot.gripper.extendGrip.setPosition(Pos);

                break;

            case 4: // INTAKE SERVO LEFT
                telemetry.addData("INTAKE", "LEFT");
                telemetry.addData("Pos: ", robot.intake.getLeftServoPos());

                Pos = robot.intake.getLeftServoPos();

                if (gamepad2.x) {
                    Pos += 0.001;
                } else if (gamepad2.b) {
                    Pos -= 0.001;
                }

                Range.clip(Pos, 0, 1);
                robot.intake.setLeftServoPos(Pos);
                break;

            case 5: // INTAKE SERVO RIGHT
                telemetry.addData("INTAKE", "RIGHT");
                telemetry.addData("Pos: ", robot.intake.getRightServoPos());

                Pos = robot.intake.getRightServoPos();

                if (gamepad2.x) {
                    Pos += 0.001;
                } else if (gamepad2.b) {
                    Pos -= 0.001;
                }

                Range.clip(Pos, 0, 1);
                robot.intake.setRightServoPos(Pos);
                break;

            case 6: // LIFT LIMIT SWITCH
                telemetry.addData("LIFT", "LIMITS");
                telemetry.addData("Pos B: ", robot.lift.liftLimitB.getState());
                telemetry.addData( "Pos T: ", robot.lift.liftLimitT.getState());
                break;

            case 7: // JEWEL ROTATE
                telemetry.addData("JEWEL", "ROTATE");
                telemetry.addData("Pos: ", robot.jewelRotServo.getPosition());

                Pos = robot.jewelRotServo.getPosition();

                if (gamepad2.x) {
                    Pos += 0.001;
                } else if (gamepad2.b) {
                    Pos -= 0.001;
                }

                Range.clip(Pos, 0, 1);
                robot.jewelRotServo.setPosition(Pos);
                break;

            case 8: // JEWEL DEPLOY
                telemetry.addData("JEWEL", "DEPLOY");
                telemetry.addData("Pos: ", robot.jewelServo.getPosition());

                Pos = robot.jewelServo.getPosition();

                if (gamepad2.x) {
                    Pos += 0.001;
                } else if (gamepad2.b) {
                    Pos -= 0.001;
                }

                Range.clip(Pos, 0, 1);
                robot.jewelServo.setPosition(Pos);
                break;

            case 9:// DISTANCE SENSOR
                telemetry.addData("DISTANCE", "SENSOR");
                telemetry.addData("Distance: ", robot.intake.distanceSensor.getDistance(DistanceUnit.CM));
                break;

            case 10: // LIFT MOTOR
                telemetry.addData("LIFT", "MOTOR");
                telemetry.addData("EncPos: ", robot.lift.liftMotor.getCurrentPosition());
                // Tune the motor PID parameters
                if (robot.lift.liftMotor instanceof DcMotorEx) {
                    DcMotorEx theLift = (DcMotorEx) robot.lift.liftMotor;
                    PIDCoefficients pid = theLift.getPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
                    telemetry.addData("p: ", pid.p);
                    telemetry.addData("i: ", pid.i);
                    telemetry.addData("d: ", pid.d);
                    telemetry.update();
                }
                if (gamepad2.right_stick_y > 0.2 || gamepad2.right_stick_y < -0.2){
                    robot.lift.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    double liftPower = -gamepad2.right_stick_y;
                    liftPower = smoothPowerCurve(deadzone(liftPower, 0.10));
                    Range.clip(liftPower, -1, 1);
                    robot.lift.liftMotor.setPower(liftPower);
                    autoLift = false;
                } else if (!autoLift) {
                    robot.lift.stopLift();
                }

                if (gamepad2.y) {
                    robot.lift.setLiftTop();
                    autoLift = true;
                }
                if (gamepad2.a) {
                    robot.lift.setLiftBtm();
                    autoLift = true;
                }

                if (gamepad2.right_bumper) {
                    resetLift = true;
                    autoLift = true;
                }

                if (resetLift) {
                    if (robot.lift.resetFloorPos()){
                        resetLift = false;
                        autoLift = false;
                    }
                }

                if (gamepad2.left_bumper) {
                    resetLiftTop = true;
                    autoLift = true;
                }

                if (resetLiftTop) {
                    if (robot.lift.resetTopPos()) {
                        resetLiftTop = false;
                        autoLift = false;
                    }
                }
                break;

            case 11: // INTAKE MOTORS
                telemetry.addData("lPower", robot.intake.lInPower);
                telemetry.addData("rPower", robot.intake.rInPower);
                telemetry.addData("Cycle: ", robot.intake.intakeCycle);
                intakeControl();
                break;

            case 12: // DRIVE TRAIN

                telemetry.addData("ldrive1: ", robot.leftDrive1.getCurrentPosition());
                telemetry.addData("ldrive2: ", robot.leftDrive2.getCurrentPosition());
                telemetry.addData("rdrive1: ", robot.rightDrive1.getCurrentPosition());
                telemetry.addData("rdrive2: ", robot.rightDrive2.getCurrentPosition());

                double left;
                double right;

                double throttle = -gamepad2.left_stick_y;
                double direction = gamepad2.right_stick_x;

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

                break;

            default:
                telemetry.addData("THIS SHOULD", "NEVER SHOW");
                break;
        }

        lastState = curState;

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

        // Give the intake a chance to adjust speeds in cycle
        robot.intake.updateInPower();   // Must be called each cycle for speed to vary properly

        // Intake IN
        if (gamepad1.right_trigger > 0.5) {
            robot.intake.setIn();
            intake = true;
        }

        // Intake STOP
        if (intake && robot.intake.detectGlyph()) {
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


