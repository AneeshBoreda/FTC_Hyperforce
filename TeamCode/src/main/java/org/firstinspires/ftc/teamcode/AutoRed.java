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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 * <p>
 * The code assumes that you do NOT have encoders on the wheels,
 * otherwise you would use: PushbotAutoDriveByEncoder;
 * <p>
 * The desired path in this example is:
 * - Drive forward for 3 SECONDS
 * - Spin right for 1.3 seconds
 * - Drive Backwards for 1 Second
 * - Stop and close the claw.
 * <p>
 * The code is written in a simple form with no optimizations.
 * However, there are several ways that this type of sequence could be streamlined,
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "Pushbot: Auto Drive By Time", group = "Pushbot")
@Disabled
public class AutoRed extends LinearOpMode {


    private DcMotorController control;
    private DcMotorController control2;
    private ServoController servoController;
    private DcMotorController control3;
    private DcMotor left_front;
    private DcMotor right_front;
    private DcMotor left_back;
    private DcMotor right_back;
    private DcMotor lift;
    private static boolean x = true;
    private static boolean y = true;
    private static boolean changeSpeed = true;
    private static double[] speeds = new double[]{0.25, 0.5, 1};
    private static int index = 2;
    private static double speed = speeds[index];

    private static boolean reset = true;
    private Servo ClawL1;
    private Servo ClawR1;
    private Servo ClawL2;
    private Servo ClawR2;
    private ColorSensor color_sensor;
    private Servo Color1;
    private Servo Color2;
    public static double threshold = 0.2;
    private ElapsedTime runtime = new ElapsedTime();

    static final int colorThreshhold = 5;
    static final double SPEED = 0.7;


    @Override
    public void runOpMode() {

        control = hardwareMap.dcMotorController.get("drive_controller");
        servoController = hardwareMap.servoController.get("servo_controller");
        ClawL1 = hardwareMap.servo.get("l1");
        ClawL2 = hardwareMap.servo.get("l2");
        ClawR1 = hardwareMap.servo.get("r1");
        ClawR2 = hardwareMap.servo.get("r2");
        Color1 = hardwareMap.servo.get("c1");
        Color2 = hardwareMap.servo.get("c2");
        left_front = hardwareMap.dcMotor.get("left_front");
        right_front = hardwareMap.dcMotor.get("right_front");
        lift = hardwareMap.dcMotor.get("lift");
        control2 = hardwareMap.dcMotorController.get("drive_controller2");


        left_back = hardwareMap.dcMotor.get("left_back");
        right_back = hardwareMap.dcMotor.get("right_back");

        control3 = hardwareMap.dcMotorController.get("lift_controller");

        color_sensor = hardwareMap.colorSensor.get("color");
        color_sensor.enableLed(true);

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        Color1.setPosition(0.25);
        Color2.setPosition(1);
        wait(0.5f);
        Color2.setPosition(0.5);
        detectcolor();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive forward for 3 seconds

        // Step 3:  Drive Backwards for 1 Second




    }

    public int detectcolor() {
        if (color_sensor.blue() - color_sensor.red() > colorThreshhold) {
            moveForward(0.5f);
        }
        else if (color_sensor.red() - color_sensor.blue() > colorThreshhold) {
            moveBackward(0.5f);
        }
        else {
            Color1.setPosition(0.75);
            Color2.setPosition(0);
            wait(0.5f);
            Color2.setPosition(0.5);
        }
    }

    public void forward() {
        left_front.setPower(-SPEED);
        left_back.setPower(-SPEED);
        right_front.setPower(SPEED);
        right_back.setPower(SPEED);
    }

    public void backward() {
        left_front.setPower(SPEED);
        left_back.setPower(SPEED);
        right_front.setPower(-SPEED);
        right_back.setPower(-SPEED);
    }

    public void left() {
        left_front.setPower(SPEED);
        left_back.setPower(SPEED);
        right_front.setPower(SPEED);
        right_back.setPower(SPEED);
    }

    public void right() {
        left_front.setPower(-SPEED);
        left_back.setPower(-SPEED);
        right_front.setPower(-SPEED);
        right_back.setPower(-SPEED);
    }


    public void wait(float time) {
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < time) {
            telemetry.addData("Running", runtime.seconds());
            telemetry.update();
        }
    }

    public void resetMotors() {
        left_front.setPower(0);
        left_back.setPower(0);
        right_front.setPower(0);
        right_back.setPower(0);
    }

    public void pause(float time) {
        resetMotors();
        wait(time);
    }

    public void moveForward(float time) {
        forward();
        wait(time);
        pause(1 / 2);
    }

    public void moveBackward(float time) {
        forward();
        wait(time);
        pause(1 / 2);
    }

    public void moveLeft(float time) {
        left();
        wait(time);
        pause(1 / 2);

    }


    public void moveRight(float time) {
        right();
        wait(time);
        pause(1 / 2);

    }


}
