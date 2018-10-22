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

import android.app.Activity;
import android.app.ActivityManager;
import android.content.ComponentName;
import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorManager;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.configuration.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.system.SystemProperties;

import org.opencv.android.*;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="RED_Autonomous17", group="2017")
public class RED_Autonomous17 extends LinearOpMode {

    private static final boolean DEBUG = false;
    private static final boolean CONTROLLERS = false;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private Servo clawLeft = null;
    private Servo clawRight = null;
    private DcMotor verticalDrive = null;
    private Servo jewelArm = null;

    private double ldrive = 0.0;
    private double rdrive = 0.0;
    private double vdrive = 0.0;
    private boolean clawClosed = false;

    static final double     COUNTS_PER_MOTOR_REV    = 112; // or 112    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.005;
    static final double     TURN_SPEED              = 0.1;
    static final double     ROBO_WIDTH = 12;

    private enum Phase
    {
        SCAN_PICTOGRAPH,
        PICK_JEWEL,
        KILL_JEWEL,
        DELIVER_1ST_GLYPH,
        DELIVER_GLYPHS
    }

    private enum Direction
    {
        LEFT,
        RIGHT,
        CENTER,
        UKNOWN
    }
    private Phase game;
    private VuforiaLocalizer vuforia;
    private VuforiaTrackable vumarkTrackable;
    private Direction vumarkDir = Direction.UKNOWN;
    private double phaseStart = 0;
    private JewelDecider jewelDecider;
    private boolean teamColorRed = true;

    // OrientationManager orientationManager;
    // OPENCV STUFF

    private void openClaws()
    {
        if (DEBUG) return;
        clawLeft.setPosition(0.0);
        clawRight.setPosition(1.0);
    }

    private void closeClaws()
    {
        if (DEBUG) return;
        clawLeft.setPosition(0.5);
        clawRight.setPosition(0.5);
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        if (!DEBUG) {
            // Initialize the hardware variables. Note that the strings used here as parameters
            // to 'get' must correspond to the names assigned during the robot configuration
            // step (using the FTC Robot Controller app on the phone).
            leftDrive = hardwareMap.get(DcMotor.class, "motorLeft");
            rightDrive = hardwareMap.get(DcMotor.class, "motorRight");
            verticalDrive = hardwareMap.get(DcMotor.class, "motorVertical");
            //clawLeft = hardwareMap.get(Servo.class, "clawLeft");
            //clawRight = hardwareMap.get(Servo.class, "clawRight");
            jewelArm = hardwareMap.get(Servo.class, "jewelArm");

            // close claws
            //clawRight.setPosition(.8);
            //clawLeft.setPosition(0.2);

            // Most robots need the motor on one side to be reversed to drive forward
            // Reverse the motor that runs backwards when connected directly to the battery
            leftDrive.setDirection(DcMotor.Direction.FORWARD);
            rightDrive.setDirection(DcMotor.Direction.REVERSE);
            verticalDrive.setDirection(DcMotor.Direction.FORWARD);


            leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        }

        // setup vuforia stuff
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AdYqe2f/////AAAAGQlysTgmVkWpo3Mq6j11chZG+wr11OCb63Vb/2wNDWpcyp7zb4UEr664Uhem/ffabxlTEMr8KQPmd3f46XXkRGunyiH5BToRfqnKEbfpZxnnY9bkc+4aSTlOl+ZLX5+7iw5LzcUT2xyK/N/7Y80p/axljhqI0QpDR3kwW90dL5N1q0D0edXv0bBUVQomVavMPT6rbf9dFBsA49bdxBS/2Axl7+3qmyzinsPhveum2Rw/35cqAoOZtoIQScnhSoAxsbhbz1lHhrXp/UppJDuauij/LezSMiGd7sWJp7o1kCV5J3M5hTcSc670Jfu5kB1bUfGmcu5GJd1+e49/hANADyCDA6UtMhDShemwFmq42xxj";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables vumarkTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        vumarkTrackable = vumarkTrackables.get(0);

        // configure team
        if (CONTROLLERS) {
            telemetry.addData("ConfigurationRed", "Press Gamepad1, button A to select team RED");
            telemetry.addData("ConfigurationBlu", "Press Gamepad1, button B to select team BLUE");
            telemetry.update();
            for (; ; ) {
                if (gamepad1.a) {
                    teamColorRed = true;
                    break;
                } else if (gamepad1.b) {
                    teamColorRed = false;
                    break;
                }
            }
        }

        telemetry.addData("TEAM", "Selected team " + (teamColorRed? "RED" : "BLUE"));
        telemetry.addData(">", "Press Play to start");
        telemetry.update();

     /*   OrientationManager orientationManager = new OrientationManager();
        orientationManager.initSensor(hardwareMap.appContext);
        orientationManager.startSensor();*/

        // close left claw
        /*clawLeft.setPosition(0.8);

        phaseStart = System.currentTimeMillis() + 1000;
        while (System.currentTimeMillis() < phaseStart);

        clawRight.setPosition(0.2);*/

        waitForStart();

        //  orientationManager.stopSensor();

        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        telemetry.update();
        game = Phase.SCAN_PICTOGRAPH;
        //game = Phase.DELIVER_1ST_GLYPH;
        game = Phase.PICK_JEWEL;

        jewelDecider = new JewelDecider(telemetry, hardwareMap.appContext);
        vumarkTrackables.activate();

        phaseStart = runtime.seconds();
        while (opModeIsActive()) {

            switch (game)
            {
                case SCAN_PICTOGRAPH:
                    if (vumarkDir == Direction.UKNOWN) {
                        ScanPictographs();
                        telemetry.addData("Vumark", "Scanning for Pictograph...");
                    }
                    else {
                        telemetry.addData("Vumark", "Pictograph Found: " + vumarkDir);
                        game = Phase.PICK_JEWEL;
                        phaseStart = runtime.seconds();
                    }
                    break;
                case PICK_JEWEL:
                    PickJewel();
                    break;
                case KILL_JEWEL:

                    break;

                case DELIVER_1ST_GLYPH:
                    Deliver1stGlyph();
                    game = Phase.KILL_JEWEL;
                    break;

                case DELIVER_GLYPHS:

                    break;
            }

            // Send calculated power to wheels
            //leftDrive.setPower(ldrive);
            //rightDrive.setPower(rdrive);

            // send power to vertical drive
            //verticalDrive.setPower(vdrive);

            /* HANDLE CLAWS */
            telemetry.update();
        }
    }

    void ScanPictographs()
    {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(vumarkTrackable);
        // wait for 2 seconds
        if (runtime.seconds() - phaseStart < 3)
            return;

        // timeout is 4 secnds
        if (runtime.seconds() - phaseStart > 7) {
            phaseStart = runtime.seconds();
            game = Phase.PICK_JEWEL;
            telemetry.addData("PICTO", "FAILED TO FIND PICTOGRAPH");
            vumarkDir = Direction.CENTER;
        }

        switch (vuMark)
        {
            case LEFT:
                vumarkDir = Direction.LEFT;
                break;
            case RIGHT:
                vumarkDir = Direction.RIGHT;
                break;
            case CENTER:
                vumarkDir = Direction.CENTER;
                break;
            default:
                // no mark in sight
        }
    }

    void PickJewel()
    {
        jewelDecider.startCapture();
        //game = Phase.DELIVER_1ST_GLYPH;

        // drop arm
        // start from 0.5, go to .75
        /****** IMPORTANT: JEWEEL START POSITION ***/
        jewelArm.setPosition(1.0);

        // wait for 1 seconds
        if (runtime.seconds() - phaseStart < 5)
            return;

        if (jewelDecider.RedJewelRight()) {
            if (teamColorRed) {
                // red jewel is right, and you are team red
                // drop arm, rotate LEFT to ckick blue jewel
                telemetry.addData("Jewel", "Rotating left");
                rotateLeft();
            }
            else {
                // red jewel is right, and you are team blue
                // drop arm, rotate RIGHT to kick red jewel
                telemetry.addData("Jewel", "Rotating right");
                rotateRight();

                // MUST ADD ROTATE LEFT AFTER
            }
        } else {
            if (teamColorRed) {
                // red jewel is left, and you are team red
                // drop arm, rotate RIGHT to kick blue jewel
                telemetry.addData("Jewel", "Rotating right");
                rotateRight();
            }
            else {
                // red jewel is left, and you are team blue
                // drop arm, rotate LEFT to kcik blue jewel
                telemetry.addData("Jewel", "Rotating left");
                rotateLeft();
            }
        }
        jewelArm.setPosition(0);
        jewelDecider.stopCapture();
        game = Phase.DELIVER_1ST_GLYPH;
        phaseStart = runtime.seconds();
    }

    void Deliver1stGlyph()
    {
        encoderDrive(1, 140, 140, 10);
        //rotateLeft();
/*
        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive.setTargetPosition(112);
        leftDrive.setPower(0.05);

        while (opModeIsActive())// && leftDrive.isBusy())
        {
            telemetry.addData("motor", "Current Position: " + leftDrive.getCurrentPosition());
            telemetry.update();
        }
        leftDrive.setPower(0);*/
        /*rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setTargetPosition(1000);
        rightDrive.setPower(0.05);

        while (opModeIsActive())// && leftDrive.isBusy())
        {
            telemetry.addData("motor", "Current Position: " + rightDrive.getCurrentPosition());
            telemetry.update();
        }
        rightDrive.setPower(0);*/

        //while (opModeIsActive());

        //encoderDrive(.1,  10,  10, 3.0);  // S1: Forward 47 Inches with 5 Sec timeout
        //encoderDrive(TURN_SPEED,   12, -12, 2.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        //encoderDrive(DRIVE_SPEED, 4, 4, 2.0);  // S3: Reverse 24 Inches with 4 Sec timeout
       /* double startTime = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - startTime < 250))
        {
            leftDrive.setPower(0.1);
            rightDrive.setPower(0.1);
            telemetry.addData("motor", "driving forward");
        }
        /*startTime = runtime.milliseconds();
        while (opModeIsActive() && (runtime.milliseconds() - startTime < 100))
        {
            leftDrive.setPower(0.2);
        }
        startTime = runtime.seconds();
        while (opModeIsActive() && (runtime.milliseconds() - startTime < 100))
        {
            openClaws();
        }
        startTime = runtime.milliseconds();
        while (opModeIsActive() && (runtime.milliseconds() - startTime < 100))
        {
            leftDrive.setPower(0.2);
            rightDrive.setPower(0.2);
        }*/
    }

    public void rotate(double angle)
    {
        double radius = ROBO_WIDTH / 2;
        double length = (angle * Math.PI * radius) / 180;

        encoderDrive(0.0001, -length, length, 10);
    }

    public void rotateLeft()
    {
        encoderDrive(0.8, 90, 90, 6);
    }

    public void rotateRight()
    {
        encoderDrive(0.8, -90, -90, 6);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftDrive.getCurrentPosition(),
                        rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftDrive.setPower(0.0);
            rightDrive.setPower(0.0);


            // Turn off RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }

   /* void rotateByGyro(float theta)
    {
        float start = orientationManager.getRoll();
        float end = start + theta;

        if (theta > 0)
        {
            while (orientationManager.getRoll() < end) {
                rightDrive.setPower(0.1);
                leftDrive.setPower(-0.1);
            }
        }
        else
        {
            while (orientationManager.getRoll() > end) {
                rightDrive.setPower(-0.1);
                leftDrive.setPower(0.1);
            }
        }
    }*/
}

