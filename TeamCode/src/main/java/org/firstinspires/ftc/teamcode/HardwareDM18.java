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

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareDM18
{
    /* Public OpMode members. */
    public DcMotor  leftDrive1   = null;
    public DcMotor  leftDrive2   = null;
    public DcMotor  rightDrive1  = null;
    public DcMotor  rightDrive2  = null;

    public DcMotor  liftMotor    = null;

    public DcMotor  intakeLeftMotor   = null;
    public DcMotor  intakeRightMotor  = null;

    public Servo    jewelServo      = null;
    public Servo    jewelRotServo   = null;
    public Servo    gripTopServo    = null;
    public Servo    gripBottomServo = null;
    public Servo    gripRotateServo = null;
    public Servo    gripExtendServo = null;

    public ColorSensor  jewelCS = null;

    public final static double JEWEL_HOME = 0.11;
    public final static double JEWEL_DEPLOY = 0.76;
    public final static double JEWEL_ROT_HOME = 0.52;
    public final static double JEWEL_ROT_FWD = 0.42;
    public final static double JEWEL_ROT_REV = 0.62;

    public final static double GRIP_TOP_OPEN = 0.0;
    public final static double GRIP_TOP_CLOSED = 1.0;
    public final static double GRIP_BOTTOM_OPEN = 0.0;
    public final static double GRIP_BOTTOM_CLOSED = 1.0;
    public final static double GRIP_ROTATE_NORMAL = 0.0;
    public final static double GRIP_ROTATE_FLIPPED = 1.0;
    public final static double GRIP_EXTEND_HOME = 0.0;
    public final static double GRIP_EXTEND_OUT = 1.0;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareDM18(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize drive-train Motors
        leftDrive1  = hwMap.dcMotor.get("ldrive1");
        leftDrive2  = hwMap.dcMotor.get("ldrive2");
        rightDrive1 = hwMap.dcMotor.get("rdrive1");
        rightDrive2 = hwMap.dcMotor.get("rdrive2");

        leftDrive1.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightDrive1.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        leftDrive2.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightDrive2.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors


        // Define and Initialize intake Motors
        intakeLeftMotor = hwMap.dcMotor.get("intakeLeft");
        intakeRightMotor = hwMap.dcMotor.get("intakeRight");

        intakeLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeRightMotor.setDirection(DcMotor.Direction.FORWARD);

        // Define lift motor

        liftMotor = hwMap.dcMotor.get("lift");

        liftMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        leftDrive1.setPower(0);
        rightDrive1.setPower(0);
        leftDrive2.setPower(0);
        rightDrive2.setPower(0);
        intakeRightMotor.setPower(0);
        intakeLeftMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        jewelServo = hwMap.servo.get("jewel");
        jewelRotServo = hwMap.servo.get("jewelRot");
        gripTopServo = hwMap.servo.get("gripTop");
        gripBottomServo = hwMap.servo.get("gripBottom");
        gripRotateServo = hwMap.servo.get("gripRotate");
        gripExtendServo = hwMap.servo.get("gripExtend");

        jewelServo.setPosition(JEWEL_HOME);
        jewelRotServo.setPosition(JEWEL_ROT_HOME);
        gripTopServo.setPosition(GRIP_TOP_OPEN);
        gripBottomServo.setPosition(GRIP_BOTTOM_OPEN);
        gripRotateServo.setPosition(GRIP_ROTATE_NORMAL);

        // Define color sensor
        jewelCS = hwMap.colorSensor.get("cs");
    }
 }

