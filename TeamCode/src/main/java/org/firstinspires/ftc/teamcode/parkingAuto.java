package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

class parkingAuto extends LinearOpMode  {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor Front_left = null;
    private DcMotor Back_left = null;
    private DcMotor Front_right = null;
    private DcMotor Back_right = null;
    //    private DcMotor armMotor = null;

    public void set_config() {
        /*
        This function set configuration matching to driver controller configuration.
        Any addition or modification of harward configuration can be directly done here

         */

        System.out.println("Config start");
        Front_left  = hardwareMap.get(DcMotor.class, "front_left_motor");
        Back_left  = hardwareMap.get(DcMotor.class, "back_left_motor");
        Front_right = hardwareMap.get(DcMotor.class, "front_right_motor");
        Back_right = hardwareMap.get(DcMotor.class, "back_right_motor");

        // setting up motor direction
        Front_left.setDirection(DcMotor.Direction.REVERSE);
        Back_left.setDirection(DcMotor.Direction.REVERSE);
        Front_right.setDirection(DcMotor.Direction.FORWARD);
        Back_right.setDirection(DcMotor.Direction.FORWARD);

        System.out.println("Config end");
    }

    // Robot functions to use during teleop
    public void moveRobot_in_teleOp_mode(){
        /*
        This function allow robot to run in teleop mode where gamepad 1 joy sticks (both left and right)
        can control robot movement
         */
        double max;
        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;
        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        // Send calculated power to wheels
        Front_left.setPower(0.5*leftFrontPower);
        Front_right.setPower(0.5*rightFrontPower);
        Back_left.setPower(0.5*leftBackPower);
        Back_right.setPower(0.5*rightBackPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        telemetry.update();

        // Movement code with gamepad ends
    }



    //Robot basic functions to use either during teleop or autonomus to control robot
    public void moveRobot(int milliseconds, float power, boolean strafe){
        /* This function is for autonomus mode to move Robot forward or backward
        INPUT:
            - milliseconds : its time in milliseconds.
                             Robot will move forward or backward for specified  milliseconds
                             values should be in integer only (no decimals)
                   - power : its power for robot movement.
                             Robot will move forward or backward with specified  power
                             power should be between -1 and 1.
                             Negative power move robot backward or strafe LEFT
                             Positive power move robot forward or strafe RIGHT


        */
        setRobotPower(power,strafe); // moving robot with specified power
        sleep(milliseconds); // keeping robot move for specified duration
        setRobotPower(0,false); // stopping robot from further moving


    }
    public void turnRobot(int milliseconds, float power){
        /* This function is for autonomus mode to turn Robot LEFT or RIGHT
        //power > 0 = left      power < 0 = right           90 degrees turn is millis 840
        INPUT:
            - milliseconds : its time in milliseconds.
                             Robot will turn LEFT or RIGHT for specified  milliseconds
                             values should be in integer only (no decimals)
                   - power : its power for robot turn.
                             Robot will turn LEFT or RIGHT with specified  power
                             power should be between -1 and 1.
                             Negative power turn robot in RIGHT direction
                             Positive power turn robot in LEFT direction


        */
        //setting up power for robot turn
        Front_left.setPower(-1 * power);
        Front_right.setPower( power);
        Back_left.setPower(-1 * power);
        Back_right.setPower( power);

        sleep(milliseconds); // keeping robot move for specified duration
        setRobotPower(0,false); // stopping robot from further moving

    }

    public void setRobotPower(float power,boolean strafe){
        /*
        This function sets power to all motor/wheels with specified power
        INPUT:
            power : its power for robot movement.
                    Robot will move forward or backward with specified  power
                    power should be between -1 and 1.
                    Negative power move robot backward or strafe LEFT
                    Positive power move robot forward or strafe RIGHT

            strafe: it's flag for strafe.
                    FALSE value will not allow robot to strafe
                    TRUE value will allow robot to strafe LEFT and Right with specified power direction
        */
        //power = power; // Reversing power due to configuration as  power<0 = forward  & power>0 = backwards.

        if (strafe){
            // When Strafe is TRUE set power accordingly to strafe robot in LEFT or RIGHT
            Front_left.setPower(-1* power);
            Back_right.setPower(-1* power);
        }
        else {
            // When Strafe is FALSE set power accordingly to strafe robot in FORWARD or BACKWARD
            Front_left.setPower(power);
            Back_right.setPower(power);
        }

        Front_right.setPower(power);
        Back_left.setPower(power);
    }


    public void runOpMode() {
        set_config();
        waitForStart();
        moveRobot(3000,0.8f,true);
    }
}

