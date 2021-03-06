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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


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

@TeleOp(name="SinglePlayer_RoboDrive17", group="2017")
public class SinglePlayer_RoboDrive17 extends LinearOpMode {

    private static final boolean NO_SWITCHES = true;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private Servo clawLeft = null;
    private Servo clawRight = null;
    private DcMotor verticalDrive = null;
    private DigitalChannel topButton = null;
    private DigitalChannel bottomButton = null;
    private Servo jewelArm;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "motorLeft");
        rightDrive = hardwareMap.get(DcMotor.class, "motorRight");

        verticalDrive = hardwareMap.get(DcMotor.class, "motorVertical");
        clawLeft = hardwareMap.get(Servo.class, "clawLeft");
        clawRight = hardwareMap.get(Servo.class, "clawRight");
        topButton = hardwareMap.get(DigitalChannel.class, "topButton");
        bottomButton = hardwareMap.get(DigitalChannel.class, "bottomButton");
        jewelArm = hardwareMap.get(Servo.class, "jewelArm");

        topButton.setMode(DigitalChannel.Mode.INPUT);
        bottomButton.setMode(DigitalChannel.Mode.INPUT);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        verticalDrive.setDirection(DcMotor.Direction.FORWARD);

        // SWITCH TEST
      /*  while (!opModeIsActive())
        {
            telemetry.addData("TOP Button: ", "" + topButton.getState());
            telemetry.addData("BOTTOM Button: ", "" + bottomButton.getState());
            telemetry.update();
        }*/

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /*   HANDLE PLAYER DRIVE */

            // Gamepad1
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            double leftPower = Range.clip(drive + turn * 2, -1.0, 1.0);
            double rightPower = Range.clip(drive - turn * 2, -1.0, 1.0);

            telemetry.addData("Turning", "Left: " + leftPower + " | Right: " + rightPower);

            // Gamepad2
            // slow down up
           // double vdrive = gamepad1.dpad_up? 0.7 : * 0.8;
            double vdrive = 0;
            if (gamepad1.dpad_up)
                vdrive = .7;
            else if (gamepad1.dpad_down)
                vdrive = -.7;

            // reachedtop is true if top button is pressed
            boolean reachedTop = false;
            boolean reachedBottom = false;
            if (!NO_SWITCHES)
            {
                reachedTop = topButton.getState();
                reachedBottom = !bottomButton.getState();
            }


            telemetry.addData("Sensors", "Reached top: " + reachedTop +  " | Reached buttom: " + reachedBottom);

            // Send calculated power to wheels
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            if (vdrive > 0 && !reachedBottom)
                // send power to vertical drive
                verticalDrive.setPower(vdrive);
            else if (vdrive < 0 && !reachedTop)
                verticalDrive.setPower(vdrive);
            else
                verticalDrive.setPower(0.0);

            /* HANDLE CLAWS */

            // range from 0.15 - 1.0
            double openness = gamepad1.right_trigger + 0.15;

            double leftclaw = openness * 0.6;
            clawLeft.setPosition(leftclaw);

            double rightclaw = 1 - (openness * 0.6);
            clawRight.setPosition(rightclaw);
            /*// left trigger will open the claw, right trigger closes it
            boolean ltrigger = gamepad2.left_trigger > 0;   // if true, open claw
            boolean rtrigger = gamepad2.right_trigger > 0;  // if false, close claw


            if (ltrigger && !rtrigger && clawClosed)
            {
                // open claw
                // case 1
                clawLeft.setPosition(0.0);
                clawRight.setPosition(1.0);
                // case 2
                // clawLeft.setPosition(1.0);
                //clawRight.setPosition(0.0);
                telemetry.addData("Claws", "Opened Claws");
                clawClosed = false;
            }
            else if (rtrigger && !ltrigger && !clawClosed)
            {
                // close claw
                clawLeft.setPosition(0.6);
                clawRight.setPosition(0.4);
                telemetry.addData("Claws", "Closed Claws");
                clawClosed = true;
            }
            else
            {
                telemetry.addData("Claws", "Do not open/close claws at same time!");
            }*/

            telemetry.update();
        }
    }
}

