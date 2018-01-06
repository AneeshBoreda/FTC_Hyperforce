
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * This class allows the driver controlled operation of the robot.
 * It controls 4 mechanum wheels and allows for strafing,
 * in addition to the normal controls of turning and forward and backward movement
 *V 1.0
 *
 *
 */

@TeleOp(name="TankDrive", group="Linear Opmode")
public class TankDrive extends OpMode
{

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
    @Override
    public void init()
    {
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
        //color_sensor.enableLed(true);

    }
    //Helper method for resetting all motors to a stop
    public void resetMotors(){
        left_front.setPower(0);
        left_back.setPower(0);
        right_front.setPower(0);
        right_back.setPower(0);
    }

    //Helper method for logging values to screens
    public void log(String main, String val){
        telemetry.addData(main, val);
    }

    @Override
    public void loop() throws IllegalArgumentException
    {

        //int red = color_sensor.red();

        //log("SayRed", Integer.toString(red));


        //int blue = color_sensor.blue();

        //log("SayBlue", Integer.toString(blue));
        if(changeSpeed&&gamepad1.dpad_up)
        {
            changeSpeed = false;
            index = Math.min(index+1, speeds.length-1);
            speed = speeds[index];
        }
        else if(changeSpeed&&gamepad1.dpad_down)
        {
            changeSpeed = false;
            index = Math.max(index-1, 0);
            speed = speeds[index];
        }
        else if(!changeSpeed&&!gamepad1.dpad_up&&!gamepad1.dpad_down)
        {
            changeSpeed = true;
        }
        log("speed", Double.toString(speed));

         /*
          *  Checks if the y value is forward or backwards.
          *  The y values are reversed so we have to negate it to logically use it.
          *
          */
        if(gamepad2.dpad_down)
        {
            lift.setPower(100);
        }
        else if(gamepad2.dpad_up)
        {
            lift.setPower(-100);
        }
        else
        {
            lift.setPower(0);
        }
        if(Math.abs(gamepad1.left_stick_y) > threshold){
            left_front.setPower(speed*Math.pow(gamepad1.left_stick_y, 3));
            left_back.setPower(speed*Math.pow(gamepad1.left_stick_y, 3));
            log("Left Stick Y", Float.toString(gamepad1.left_stick_y));
        }


         /*
          *  Checks if the y value is forward or backwards.
          *
          */
        if(Math.abs(gamepad1.right_stick_y) > threshold){
            right_front.setPower(speed*Math.pow(-gamepad1.right_stick_y, 3));
            right_back.setPower(speed*Math.pow(-gamepad1.right_stick_y, 3));
            log("Right Stick Y", Float.toString(-gamepad1.right_stick_y));

        }


        /*
         *  Used for strafing, sets the power to allow sideways movement when shifted to the right
         *
         */
        if(Math.abs(gamepad1.left_stick_x) > threshold && Math.abs(gamepad1.right_stick_x) > threshold){
            left_front.setPower(Math.pow(-gamepad1.left_stick_x, 3));
            left_back.setPower(Math.pow(gamepad1.left_stick_x, 3));
            right_front.setPower(Math.pow(-gamepad1.right_stick_x, 3));
            right_back.setPower(Math.pow(gamepad1.right_stick_x, 3));
            log("Sideways", Float.toString(-gamepad1.right_stick_x));
            log("Sideways", Float.toString(-gamepad1.left_stick_x));

        }
        /*
         *  Resets motors when the control isn't moving
         *
         */
        if(Math.abs(gamepad1.left_stick_x) < threshold && Math.abs(gamepad1.right_stick_x) < threshold
                && Math.abs(gamepad1.left_stick_y) < threshold && Math.abs(gamepad1.right_stick_y) < threshold){
            resetMotors();
        }
        /*
         * Code for opening and closing the claw
         *
         */
        if(gamepad2.y && y)
        {
            y = false;
            x = true;
            reset = true;
            log("x", Boolean.toString(x));
            log("y", Boolean.toString(y));
            log("reset", Boolean.toString(reset));
            ClawL1.setPosition(0.65);
            ClawR1.setPosition(0.35);
            ClawL2.setPosition(0.65);
            ClawR2.setPosition(0.35);
        }
        else if(gamepad2.x && x)
        {

            y = true;
            x = false;
            reset = true;
            log("x", Boolean.toString(x));
            log("y", Boolean.toString(y));
            log("reset", Boolean.toString(reset));
            ClawL1.setPosition(0.8);
            ClawR1.setPosition(0.2);
            ClawL2.setPosition(0.8);
            ClawR2.setPosition(0.2);
        }
        else if(gamepad2.b && reset)
        {

            x = true;
            y = true;
            reset = false;
            log("x", Boolean.toString(x));
            log("y", Boolean.toString(y));
            log("reset", Boolean.toString(reset));
            ClawL1.setPosition(0.4);
            ClawR1.setPosition(0.6);
            ClawL2.setPosition(0.4);
            ClawR2.setPosition(0.6);
        }




    }

}

