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

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;



/**
 * This file illustrates the concept of driving up to a line and then stopping.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code shows using two different light sensors:
 *   The Primary sensor shown in this code is a legacy NXT Light sensor (called "sensor_light")
 *   Alternative "commented out" code uses a MR Optical Distance Sensor (called "sensor_ods")
 *   instead of the LEGO sensor.  Chose to use one sensor or the other.
 *
 *   Setting the correct WHITE_THRESHOLD value is key to stopping correctly.
 *   This should be set half way between the light and dark values.
 *   These values can be read on the screen once the OpMode has been INIT, but before it is STARTED.
 *   Move the senso on asnd off the white line and not the min and max readings.
 *   Edit this code to make WHITE_THRESHOLD half way between the min and max.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Auto Jewel Blue", group="DM18")
//@Disabled
public class Auto_Jewel_Blue extends LinearOpMode {


    /* Declare OpMode members. */
    HardwareDM18         robot   = new HardwareDM18 ();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();




    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;




    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.8;     // Nominal speed for auto moves.
    static final double     DRIVE_SPEED_SLOW        = 0.65;     // Slower speed where required
    static final double     TURN_SPEED              = 0.8;     // Turn speed

    static final double     HEADING_THRESHOLD       = 2 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.011;   // Larger is more responsive, but also less accurate
    static final double     P_DRIVE_COEFF_1         = 0.03;  // Larger is more responsive, but also less accurate
    static final double     P_DRIVE_COEFF_2         = 0.02;

    // Variables used for reading Gyro
    Orientation             angles;
    double                  headingBias = 0.0;            // Gyro heading adjustment

    // Color Sensor Values
    static final double     BLUE_MIN                = -180.0;
    static final double     BLUE_MAX                = -140.0;
    static final double     RED_MIN                 = -20.0;
    static final double     RED_MAX                 = 20.0;

    // Variable for number of glyphs collected

    int glyphsCollected = 0;

    // Storage for reading adaFruit color sensor for beacon sensing
    // adaHSV is an array that will hold the hue, saturation, and value information.
    float[] adaHSV = {0F, 0F, 0F};
    // adaValues is a reference to the adaHSV array.
    final float adaValues[] = adaHSV;

    /**
     * The main routine of the OpMode.
     *
     * @throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException {

        // Init the robot hardware -- including gyro and range finder
        robot.init(hardwareMap, true);

        // Force reset the drive train encoders.  Do it twice as sometimes this gets missed due to USB congestion
        robot.setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        robot.setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        idle();
        robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RobotLog.i("DM10337 -- Drive train encoders reset");

        RobotLog.i("DM10337- Finished Init");

        // Show telemetry for gyro status
        telemetry.addData("IMU calibrated: ", robot.adaGyro.isSystemCalibrated());
        telemetry.addData("IMU Gyro calibrated:  ", robot.adaGyro.isGyroCalibrated());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)

        // Set a flipTimer of how often to update gyro status telemetry
        ElapsedTime updateGyroStatTimer = new ElapsedTime();
        updateGyroStatTimer.reset();


        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey = "AUt1Gf7/////AAAAGYGzxJPM7kkwgvjUoJXLxlZEJ9MMmT+9SHOR4KceMDfzyFZGLeLoQnB63pcWT0XT87KgeSQCKD7cV0LtdqEkgGGC3TFqW5W5d/VBctIrBtRd8hdIydaIJslqHa9G7itg1/lr94VhCA8niUgqpBB8dPx8PQ6zrUHBdJrQnczLvmxOSrbDzM6vCgmqyK1sfEsmN+GBux+0as/1a1Y6TIhHQDKZ7mCi8+nBB2mZYjtVVD3sDpPcEulol15QUhLli4PVFY1JB64jc2YqHX7IigSXbFQW1+hRMusVN5ZQjTxb1/OLqQsLQIApblHRYKO6896HYQwdHTQET6H5IFFEP6vgeFnfNk3Mv3VgabjOeT3IaDzG";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        while (!isStarted()) {
            if (updateGyroStatTimer.milliseconds() >= 500) {
                // Update telemetry every 0.5 seconds
                telemetry.addData("IMU calibrated: ", robot.adaGyro.isSystemCalibrated());
                telemetry.addData("IMU Gyro calibrated:  ", robot.adaGyro.isGyroCalibrated());

                // Do a gyro read to keep it "fresh"
                telemetry.addData("Gyro heading: ", readGyro());
                telemetry.update();

                // And reset the flipTimer
                updateGyroStatTimer.reset();
            }
            idle();
        }

        relicTrackables.activate();

        telemetry.clearAll();

        RobotLog.i("DM10337- Auto Pressed Start");
        // Step through each leg of the path,

        // Make sure the gyro is zeroed
        zeroGyro();

        RobotLog.i("DM10337 - Gyro bias set to " + headingBias);


               /*

         JEWEL CODE

          */

        double armPos = robot.jewelServo.getPosition();
        double armIncr = (robot.JEWEL_DEPLOY - armPos)/25;
        while (armPos < robot.JEWEL_DEPLOY) {
            armPos += armIncr;
            robot.jewelServo.setPosition(armPos);
            sleep(20);
        }

        robot.jewelCS.enableLed(true);
        sleep(1500);

        int jewelColor = JewelColor();

        // Check if we see blue or red
        if (jewelColor == -1) {
            // We see red
            robot.jewelRotServo.setPosition(iAmBlue()?robot.JEWEL_ROT_REV:robot.JEWEL_ROT_FWD);
            sleep(500);
        } else if (jewelColor == 1) {
            // We see blue
            robot.jewelRotServo.setPosition(iAmBlue()?robot.JEWEL_ROT_FWD:robot.JEWEL_ROT_REV);
            sleep(500);
        } else {
            // We saw neither blue nor red so do nothing
            sleep(500);
        }

        // Reset jewel arm
        robot.jewelRotServo.setPosition(robot.JEWEL_ROT_HOME);
        armPos = robot.jewelServo.getPosition();
        armIncr = (robot.JEWEL_HOME - armPos)/25;
        while (armPos > robot.JEWEL_HOME) {
            armPos += armIncr;
            robot.jewelServo.setPosition(armPos);
            sleep(20);
        }
        //robot.jewelServo.setPosition(robot.JEWEL_HOME);
        sleep(500);
        robot.jewelCS.enableLed(false);


        // Lift preloaded glyph to mid position for driving
        robot.lift.setLiftMid();

        /*

        CRYPTO CODE

         */

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        telemetry.addData("VuMark", "%s visible", vuMark);


        if (vuMark == RelicRecoveryVuMark.CENTER || vuMark == RelicRecoveryVuMark.UNKNOWN) {
            // Drive forward to lineup with center cryptoglyph
            if (iAmBlue()) {
                encoderDrive(0.5, 32.0, 5.0, false, 0.0);
            } else {
                encoderDrive(0.5, -36.0, 5.0, false, 0.0);
            }
            // Turn toward center cryptoglyph
            gyroTurn(0.8, 90, P_TURN_COEFF);
            // Drive closer to center cryptoglyph
            encoderDrive(0.5, 9.5, 3.0, false, 90);
            // Outake glyph
            robot.gripper.setExtendOut();
            robot.lift.setLiftBtm();
            idleWhile(robot.gripper.isExtending() || !robot.lift.reachedFloor());
            robot.gripper.setBothPartialOpen();
            idleWhile(robot.gripper.isMoving());
            robot.gripper.setExtendIn();
        }
        if (vuMark == RelicRecoveryVuMark.RIGHT) {
            // Drive forward to lineup with center cryptoglyph
            if (iAmBlue()) {
                encoderDrive(0.5, 32.0+7.5, 5.0, false, 0.0);
            } else {
                encoderDrive(0.5, -36.0+7.5, 5.0, false, 0.0);
            }
            // Turn toward center cryptoglyph
            gyroTurn(0.8, 90, P_TURN_COEFF);
            // Drive closer to center cryptoglyph
            encoderDrive(0.5, 9.5, 3.0, false, 90);
            // Outake glyph
            robot.gripper.setExtendOut();
            robot.lift.setLiftBtm();
            idleWhile(robot.gripper.isExtending() || !robot.lift.reachedFloor());
            robot.gripper.setBothPartialOpen();
            idleWhile(robot.gripper.isMoving());
            robot.gripper.setExtendIn();
        }
        if (vuMark == RelicRecoveryVuMark.LEFT) {
            // Drive forward to lineup with center cryptoglyph
            if (iAmBlue()) {
                encoderDrive(0.5, 32.0-7.5, 5.0, false, 0.0);
            } else {
                encoderDrive(0.5, -36.0-7.5, 5.0, false, 0.0);
            }
            // Turn toward center cryptoglyph
            gyroTurn(0.8, 90, P_TURN_COEFF);
            // Drive closer to center cryptoglyph
            encoderDrive(0.5, 9.5, 3.0, false, 90);
            // Outake glyph
            robot.gripper.setExtendOut();
            robot.lift.setLiftBtm();
            idleWhile(robot.gripper.isExtending() || !robot.lift.reachedFloor());
            robot.gripper.setBothPartialOpen();
            idleWhile(robot.gripper.isMoving());
            robot.gripper.setExtendIn();
        }


        robot.lift.resetFloorPos();
        // Drive back, but stay in safe zone
        encoderDrive(0.6, -15.0, 3.0, true, 90);

        sleep(500);

        gyroTurn(0.8, -90, P_TURN_COEFF);

        sleep (500);

        int left1Pos = robot.leftDrive1.getCurrentPosition();
        int left2Pos = robot.leftDrive2.getCurrentPosition();
        int right1Pos = robot.rightDrive1.getCurrentPosition();
        int right2Pos = robot.rightDrive2.getCurrentPosition();

        // Attempt to collect glyph into intake
        collectGlyph(0.25, 5, true, -90);

        // Determine if glyph is in intake. If so, auto load first glyph and take account for it.
        if (robot.intake.detectGlyph()) {
            autoLoadFirstGlyph();
            glyphsCollected += 1;
        }

        // Attempt to collect glyph into intake
        collectGlyph(0.25, 2,true, -90);

        // Determine if glyph is in intake. If so, auto load glyph as first or second depending on whether one was previously loaded or not.
        if (robot.intake.detectGlyph() && glyphsCollected == 1) {
            autoLoadSecondGlyph();
        } else if (robot.intake.detectGlyph() && glyphsCollected == 0) {
            autoLoadFirstGlyph();
        }

        // Return to original location - safe zone
        returnToPosition(0.5, left1Pos, left2Pos, right1Pos, right2Pos, 5.0, true, -90);
        sleep(500);

        // If one or more glyphs are loaded, attempt to score them.
        if (glyphsCollected > 0){
            gyroTurn(0.8, 90, P_TURN_COEFF);

            sleep (500);

            encoderDrive(0.6, 15, 3, true, 90);

            sleep (500);

            robot.lift.setLiftTop();

            idleWhile(!robot.lift.reachedFloor());

            robot.gripper.setExtendOut();

            idleWhile(robot.gripper.isExtending());

            robot.gripper.setBothPartialOpen();

            sleep(250);

            encoderDrive(0.5, -10, 3, false, 180);

            robot.gripper.setExtendIn();
        }





        RobotLog.i("DM10337- Finished last move of auto");


        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *
     */


    /**
     * Abbreviated call to encoderDrive w/o range aggressive turning or finding adjustments
     *
     * @param speed
     * @param distance
     * @param timeout
     * @param useGyro
     * @param heading
     * @throws InterruptedException
     */
    public void encoderDrive(double speed,
                             double distance,
                             double timeout,
                             boolean useGyro,
                             double heading) throws InterruptedException {
        encoderDrive(speed, distance, timeout, useGyro, heading, false, false, 0.0);
    }


    /**
     * Abbreviated call to encoderDrive w/o range finding adjustments
     *
     * @param speed
     * @param distance
     * @param timeout
     * @param useGyro
     * @param heading
     * @param aggressive
     * @throws InterruptedException
     */
    public void encoderDrive(double speed,
                             double distance,
                             double timeout,
                             boolean useGyro,
                             double heading,
                             boolean aggressive) throws InterruptedException {
        encoderDrive(speed, distance, timeout, useGyro, heading, aggressive, false, 0.0);
    }

    /**
     *
     * Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     *
     * @param speed                 Motor power (0 to 1.0)
     * @param distance              Inches
     * @param timeout               Seconds
     * @param useGyro               Use gyro to keep/curve to an absolute heading
     * @param heading               Heading to use
     * @throws InterruptedException
     */
    public void encoderDrive(double speed,
                             double distance,
                             double timeout,
                             boolean useGyro,
                             double heading,
                             boolean aggressive,
                             boolean userange,
                             double maintainRange) throws InterruptedException {

        // Calculated encoder targets
        int newLFTarget;
        int newRFTarget;
        int newLRTarget;
        int newRRTarget;

        // The potentially adjusted current target heading
        double curHeading = heading;

        // Speed ramp on start of move to avoid wheel slip
        final double MINSPEED = 0.30;           // Start at this power
        final double SPEEDINCR = 0.015;         // And increment by this much each cycle
        double curSpeed;                        // Keep track of speed as we ramp

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            RobotLog.i("DM10337- Starting encoderDrive speed:" + speed +
                    "  distance:" + distance + "  timeout:" + timeout +
                    "  useGyro:" + useGyro + " heading:" + heading + "  maintainRange: " + maintainRange);

            // Calculate "adjusted" distance  for each side to account for requested turn during run
            // Purpose of code is to have PIDs closer to finishing even on curved moves
            // This prevents jerk to one side at stop
            double leftDistance = distance;
            double rightDistance = distance;
            if (useGyro) {
                // We are gyro steering -- are we requesting a turn while driving?
                double headingChange = getError(curHeading) * Math.signum(distance);
                if (Math.abs(headingChange) > 5.0) {
                    //Heading change is significant enough to account for
                    if (headingChange > 0.0) {
                        // Assume 16 inch wheelbase
                        // Add extra distance to the wheel on outside of turn
                        rightDistance += Math.signum(distance) * 2 * 3.1415 * 16 * headingChange / 360.0;
                        RobotLog.i("DM10337 -- Turn adjusted R distance:" + rightDistance);
                    } else {
                        // Assume 16 inch wheelbase
                        // Add extra distance from the wheel on inside of turn
                        // headingChange is - so this is increasing the left distance
                        leftDistance -= Math.signum(distance) * 2 * 3.1415 * 16 * headingChange / 360.0;
                        RobotLog.i("DM10337 -- Turn adjusted L distance:" + leftDistance);
                    }
                }
            }

            // Determine new target encoder positions, and pass to motor controller
            newLFTarget = robot.leftDrive1.getCurrentPosition() + (int)(leftDistance * robot.COUNTS_PER_INCH);
            newLRTarget = robot.leftDrive2.getCurrentPosition() + (int)(leftDistance * robot.COUNTS_PER_INCH);
            newRFTarget = robot.rightDrive1.getCurrentPosition() + (int)(rightDistance * robot.COUNTS_PER_INCH);
            newRRTarget = robot.rightDrive2.getCurrentPosition() + (int)(rightDistance * robot.COUNTS_PER_INCH);

            while(robot.leftDrive1.getTargetPosition() != newLFTarget){
                robot.leftDrive1.setTargetPosition(newLFTarget);
                sleep(1);
            }
            while(robot.rightDrive1.getTargetPosition() != newRFTarget){
                robot.rightDrive1.setTargetPosition(newRFTarget);
                sleep(1);
            }
            while(robot.leftDrive2.getTargetPosition() != newLRTarget){
                robot.leftDrive2.setTargetPosition(newLRTarget);
                sleep(1);
            }
            while(robot.rightDrive2.getTargetPosition() != newRRTarget){
                robot.rightDrive2.setTargetPosition(newRRTarget);
                sleep(1);
            }

            // Turn On motors to RUN_TO_POSITION
            robot.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            speed = Math.abs(speed);    // Make sure its positive
            curSpeed = Math.min(MINSPEED,speed);

            // Set the motors to the starting power
            robot.leftDrive1.setPower(Math.abs(curSpeed));
            robot.rightDrive1.setPower(Math.abs(curSpeed));
            robot.leftDrive2.setPower(Math.abs(curSpeed));
            robot.rightDrive2.setPower(Math.abs(curSpeed));

            // keep looping while we are still active, and there is time left, until at least 1 motor reaches target
            while (opModeIsActive() &&
                    (runtime.seconds() < timeout) &&
                    robot.leftDrive1.isBusy() &&
                    robot.leftDrive2.isBusy() &&
                    robot.rightDrive1.isBusy() &&
                    robot.rightDrive2.isBusy()) {

                // Ramp up motor powers as needed
                if (curSpeed < speed) {
                    curSpeed += SPEEDINCR;
                }
                double leftSpeed = curSpeed;
                double rightSpeed = curSpeed;

                // Doing gyro heading correction?
                if (useGyro){

                    // adjust relative speed based on heading
                    double error = getError(curHeading);
                    double steer = getSteer(error,
                            (aggressive?P_DRIVE_COEFF_1:P_DRIVE_COEFF_2));

                    // if driving in reverse, the motor correction also needs to be reversed
                    if (distance < 0)
                        steer *= -1.0;

                    // Adjust motor powers for heading correction
                    leftSpeed -= steer;
                    rightSpeed += steer;

                    // Normalize speeds if any one exceeds +/- 1.0;
                    double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                    if (max > 1.0)
                    {
                        leftSpeed /= max;
                        rightSpeed /= max;
                    }

                }

                // And rewrite the motor speeds
                robot.leftDrive1.setPower(Math.abs(leftSpeed));
                robot.rightDrive1.setPower(Math.abs(rightSpeed));
                robot.leftDrive2.setPower(Math.abs(leftSpeed));
                robot.rightDrive2.setPower(Math.abs(rightSpeed));

                // Allow time for other processes to run.
                idle();
            }


            RobotLog.i("DM10337- encoderDrive done" +
                    "  lftarget: " +newLFTarget + "  lfactual:" + robot.leftDrive1.getCurrentPosition() +
                    "  lrtarget: " +newLRTarget + "  lractual:" + robot.leftDrive2.getCurrentPosition() +
                    "  rftarget: " +newRFTarget + "  rfactual:" + robot.rightDrive1.getCurrentPosition() +
                    "  rrtarget: " +newRRTarget + "  rractual:" + robot.rightDrive2.getCurrentPosition() +
                    "  heading:" + readGyro());

            // Stop all motion;
            robot.leftDrive1.setPower(0);
            robot.rightDrive1.setPower(0);
            robot.leftDrive2.setPower(0);
            robot.rightDrive2.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (  double speed, double angle, double coefficient) {

        RobotLog.i("DM10337- gyroTurn start  speed:" + speed +
                "  heading:" + angle);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, coefficient)) {
            // Allow time for other processes to run.
            // onHeading() does the work of turning us
            idle();
        }

        RobotLog.i("DM10337- gyroTurn done   heading actual:" + readGyro());
    }


    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            // Close enough so no need to move
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            // Calculate motor powers
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.leftDrive1.setPower(leftSpeed);
        robot.rightDrive1.setPower(rightSpeed);
        robot.leftDrive2.setPower(leftSpeed);
        robot.rightDrive2.setPower(rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;


        // calculate error in -179 to +180 range  (
        robotError = targetAngle - readGyro();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }


    /**
     * Record the current heading and use that as the 0 heading point for gyro reads
     * @return
     */
    void zeroGyro() {
        angles = robot.adaGyro.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        headingBias = angles.firstAngle;
    }


    /**
     * Read the current heading direction.  Use a heading bias if we recorded one at start to account for drift during
     * the init phase of match
     *
     * @return      Current heading (Z axis)
     */
    double readGyro() {
        angles = robot.adaGyro.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return angles.firstAngle - headingBias;
    }

    /**
     * Always returns true as we are blue.
     *
     * Red OpMode would extend this class and Override this single method.
     *
     * @return          Always true
     */

    public int JewelColor () {

        // Return 1 for Blue and -1 for Red
        // convert the RGB adaValues to HSV adaValues.
        Color.RGBToHSV((robot.jewelCS.red() * 255) / 800, (robot.jewelCS.green() * 255) / 800,
                (robot.jewelCS.blue() * 255) / 800, adaHSV);

        // Normalize hue to -180 to 180 degrees
        if (adaHSV[0] > 180.0) {
            adaHSV[0] -= 360.0;
        }

        telemetry.addData("Hue: ", adaHSV[0]);
        telemetry.update();
        // Check for blue
        if ((adaHSV[0] > BLUE_MIN) && (adaHSV[0] < BLUE_MAX)) {
            // we see blue so return 1.0
            RobotLog.i("DM10337- Jewel color found blue alpha. Hue:" + adaHSV[0] );
            return 1;
        }

        // Check for red
        if (adaHSV[0] > RED_MIN && adaHSV[0] < RED_MAX) {
            //telemetry.addData("beacon", -1);
            //telemetry.update();
            RobotLog.i("DM10337- Jewel color found red. Hue:" + adaHSV[0] );
            return -1;
        }

        RobotLog.i("DM10337- Jewel color found neither. Hue:" + adaHSV[0] );
        return 0;         // We didn't see either color so don't know
    }

    /**
     * Robot drives forward while attempting to collect glyph.
     * Robot attempts to maintain heading throughout collection procress
     * Robot stops once glyph is retrieved
     **/
    public void collectGlyph (double speed, int timeout, boolean useGyro, double heading) {

        robot.intake.setClosed();
        robot.intake.setIn();

        // The potentially adjusted current target heading
        double curHeading = heading;

        // Speed ramp on start of move to avoid wheel slip
        final double MINSPEED = 0.30;           // Start at this power
        final double SPEEDINCR = 0.015;         // And increment by this much each cycle
        double curSpeed;                        // Keep track of speed as we ramp

        // reset the timeout time and start motion.
        runtime.reset();

        speed = Math.abs(speed);    // Make sure its positive
        curSpeed = Math.min(MINSPEED,speed);

        // Set the motors to the starting power
        robot.leftDrive1.setPower(Math.abs(curSpeed));
        robot.rightDrive1.setPower(Math.abs(curSpeed));
        robot.leftDrive2.setPower(Math.abs(curSpeed));
        robot.rightDrive2.setPower(Math.abs(curSpeed));

        // keep looping while we are still active, and there is time left, until distance sensor detects glyph in intake

        boolean stop = false;

        while (opModeIsActive() && (runtime.seconds() < timeout) && !stop) {

            if (robot.intake.detectGlyph()) {
                stop = true;
            }

            // Ramp up motor powers as needed
            if (curSpeed < speed) {
                curSpeed += SPEEDINCR;
            }
            double leftSpeed = curSpeed;
            double rightSpeed = curSpeed;

            // Doing gyro heading correction?
            if (useGyro){

                // adjust relative speed based on heading
                double error = getError(curHeading);
                double steer = getSteer(error, P_DRIVE_COEFF_1);

                // if driving in reverse, the motor correction also needs to be reversed
                //if (distance < 0)
                //    steer *= -1.0;

                // Adjust motor powers for heading correction
                leftSpeed -= steer;
                rightSpeed += steer;

                // Normalize speeds if any one exceeds +/- 1.0;
                double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

            }

            // And rewrite the motor speeds
            robot.leftDrive1.setPower(Math.abs(leftSpeed));
            robot.rightDrive1.setPower(Math.abs(rightSpeed));
            robot.leftDrive2.setPower(Math.abs(leftSpeed));
            robot.rightDrive2.setPower(Math.abs(rightSpeed));

            // Allow time for other processes to run.
            idle();
        }

        robot.intake.setStop();
        robot.leftDrive1.setPower(0.0);
        robot.leftDrive2.setPower(0.0);
        robot.rightDrive1.setPower(0.0);
        robot.rightDrive2.setPower(0.0);
    }

    /**
     * Robot returns to designatied encoder position
     **/
    public void returnToPosition(double speed,
                                 int left1Pos,
                                 int left2Pos,
                                 int right1Pos,
                                 int right2Pos,
                                 double timeout,
                                 boolean useGyro,
                                 double heading) throws InterruptedException {


        // The potentially adjusted current target heading
        double curHeading = heading;

        // Speed ramp on start of move to avoid wheel slip
        final double MINSPEED = 0.30;           // Start at this power
        final double SPEEDINCR = 0.015;         // And increment by this much each cycle
        double curSpeed;                        // Keep track of speed as we ramp

        int averageOriginalPos = (robot.leftDrive1.getCurrentPosition() + robot.leftDrive2.getCurrentPosition() + robot.rightDrive1.getCurrentPosition() + robot.rightDrive2.getCurrentPosition())/4;
        int averageNewPos = (left1Pos + left2Pos + right1Pos + right2Pos)/4;
        int difference = averageNewPos - averageOriginalPos;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {


            while(robot.leftDrive1.getTargetPosition() != left1Pos){
                robot.leftDrive1.setTargetPosition(left1Pos);
                sleep(1);
            }
            while(robot.rightDrive1.getTargetPosition() != right1Pos){
                robot.rightDrive1.setTargetPosition(right1Pos);
                sleep(1);
            }
            while(robot.leftDrive2.getTargetPosition() != left2Pos){
                robot.leftDrive2.setTargetPosition(left2Pos);
                sleep(1);
            }
            while(robot.rightDrive2.getTargetPosition() != right2Pos){
                robot.rightDrive2.setTargetPosition(right2Pos);
                sleep(1);
            }

            // Turn On motors to RUN_TO_POSITION
            robot.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            speed = Math.abs(speed);    // Make sure its positive
            curSpeed = Math.min(MINSPEED,speed);

            // Set the motors to the starting power
            robot.leftDrive1.setPower(Math.abs(curSpeed));
            robot.rightDrive1.setPower(Math.abs(curSpeed));
            robot.leftDrive2.setPower(Math.abs(curSpeed));
            robot.rightDrive2.setPower(Math.abs(curSpeed));

            // keep looping while we are still active, and there is time left, until at least 1 motor reaches target
            while (opModeIsActive() &&
                    (runtime.seconds() < timeout) &&
                    robot.leftDrive1.isBusy() &&
                    robot.leftDrive2.isBusy() &&
                    robot.rightDrive1.isBusy() &&
                    robot.rightDrive2.isBusy()) {

                // Ramp up motor powers as needed
                if (curSpeed < speed) {
                    curSpeed += SPEEDINCR;
                }
                double leftSpeed = curSpeed;
                double rightSpeed = curSpeed;

                // Doing gyro heading correction?
                if (useGyro){

                    // adjust relative speed based on heading
                    double error = getError(curHeading);
                    double steer = getSteer(error, P_DRIVE_COEFF_1);

                    // if driving in reverse, the motor correction also needs to be reversed
                    if (difference < 0)
                        steer *= -1.0;

                    // Adjust motor powers for heading correction
                    leftSpeed -= steer;
                    rightSpeed += steer;

                    // Normalize speeds if any one exceeds +/- 1.0;
                    double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                    if (max > 1.0)
                    {
                        leftSpeed /= max;
                        rightSpeed /= max;
                    }

                }

                // And rewrite the motor speeds
                robot.leftDrive1.setPower(Math.abs(leftSpeed));
                robot.rightDrive1.setPower(Math.abs(rightSpeed));
                robot.leftDrive2.setPower(Math.abs(leftSpeed));
                robot.rightDrive2.setPower(Math.abs(rightSpeed));

                // Allow time for other processes to run.
                idle();
            }


            RobotLog.i("DM10337- encoderDrive done" +
                    "  lftarget: " +left1Pos + "  lfactual:" + robot.leftDrive1.getCurrentPosition() +
                    "  lrtarget: " +left2Pos+ "  lractual:" + robot.leftDrive2.getCurrentPosition() +
                    "  rftarget: " +right1Pos+ "  rfactual:" + robot.rightDrive1.getCurrentPosition() +
                    "  rrtarget: " +right2Pos+ "  rractual:" + robot.rightDrive2.getCurrentPosition() +
                    "  heading:" + readGyro());

            // Stop all motion;
            robot.leftDrive1.setPower(0);
            robot.rightDrive1.setPower(0);
            robot.leftDrive2.setPower(0);
            robot.rightDrive2.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public boolean iAmBlue() {
        return true;
    }

    public void autoLoadFirstGlyph() {

        robot.gripper.setBtmClosed();

        idleWhile(robot.gripper.btmIsMoving());

        robot.intake.setOpen();

        idleWhile(robot.intake.isMoving());

        robot.lift.setLiftTop();

        idleWhile(!robot.lift.reachedFloor());

        robot.gripper.flip();

        idleWhile(robot.gripper.isFlipping());

        robot.lift.setLiftBtm();

        idleWhile(!robot.lift.reachedFloor());

        robot.lift.resetFloorPos();

        idleWhile(!robot.lift.resetFloorPos());

    }

    public void autoLoadSecondGlyph() {

        robot.gripper.setBtmClosed();

        idleWhile(robot.gripper.btmIsMoving());

        robot.intake.setOpen();

        idleWhile(robot.intake.isMoving());

        robot.lift.setLiftTop();

        idleWhile(!robot.lift.reachedFloor());
    }

    public void idleWhile(boolean function){
        while (function) {
            idle();
        }
    }
    public boolean waitForSwitch() {
        while (!gamepad1.a) {
            idle();
        }
        return true;
    }
}
