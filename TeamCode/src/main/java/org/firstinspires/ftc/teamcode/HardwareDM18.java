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

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;


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

    public Servo    jewelServo       = null;
    public Servo    jewelRotServo    = null;

    public ColorSensor  jewelCS = null;

    BNO055IMU adaGyro;

    public final static double JEWEL_HOME = 0.027;
    public final static double JEWEL_DEPLOY = 0.76;
    public final static double JEWEL_ROT_HOME = 0.52;
    public final static double JEWEL_ROT_FWD = 0.62;
    public final static double JEWEL_ROT_REV = 0.42;


    /* Drive train constants */
    static final int        COUNTS_PER_MOTOR_REV    = 7 ;    // Neverrest
    static final double     DRIVE_GEAR_REDUCTION    = 19.2 * 72 / 48 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (4 * COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);


    // Intake
    public Intake intake = new Intake();


    // Gripper
    public Gripper gripper = new Gripper();

    // Lift
    public Lift lift = new Lift();

    /* Gripper state variables */
    Servo topGrip = null;
    Servo botGrip = null;
    boolean isGripFlipped = false;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareDM18(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, boolean initGripper) {
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

        // Setup intake mapped to hardware
        intake.init(hwMap, "intakeLeft", "intakeRight", "ils", "irs", "ds");

        if (initGripper){
            // Setup gripper mapped to hardware
            gripper.init(hwMap, "gripP", "gripB", "gripRotate", "gripExtend");
            // Set gripper not flipped and closed
            gripper.setFlipped(false);
            gripper.setBothClosed();

        }

        // Setup lift mapped to hardware
        lift.init(hwMap, "lift", "llb", "llt");

        // Set all motors to zero power
        leftDrive1.setPower(0);
        rightDrive1.setPower(0);
        leftDrive2.setPower(0);
        rightDrive2.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        // Define and initialize ALL installed servos.
        jewelServo = hwMap.servo.get("jewel");
        jewelRotServo = hwMap.servo.get("jewelRot");

        // Set init positions of servos
        jewelServo.setPosition(JEWEL_HOME);
        jewelRotServo.setPosition(JEWEL_ROT_HOME);


        // Set intake to closed and stopped
        intake.setClosed();
        intake.setStop();

        // Define color sensor
        jewelCS = hwMap.colorSensor.get("cs");

        AdafruitBNO055IMU.Parameters parameters = new AdafruitBNO055IMU.Parameters();
        parameters.angleUnit           = AdafruitBNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = AdafruitBNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "adaGyro".
        adaGyro = hwMap.get(BNO055IMU.class, "gyro");
        adaGyro.initialize(parameters);


    }


    /**
     *
     * @param mode  RunMode to set the drive train to (e.g. w/ or w/o encoders)
     *
     * Sets all drive train motors to the designated mode
     */
    public void setDriveMode(DcMotor.RunMode mode) {
        leftDrive1.setMode(mode);
        leftDrive2.setMode(mode);
        rightDrive1.setMode(mode);
        rightDrive2.setMode(mode);
    }

    /**
     *
     * @param behavior  ZeroPower Behavior to set on motors -- brake or float
     *
     * Sets all drive train motors to float or brake mode as directed
     *
     */
    public void setDriveZeroPower(DcMotor.ZeroPowerBehavior behavior) {
        leftDrive1.setZeroPowerBehavior(behavior);
        leftDrive2.setZeroPowerBehavior(behavior);
        rightDrive1.setZeroPowerBehavior(behavior);
        rightDrive2.setZeroPowerBehavior(behavior);

    }
}

