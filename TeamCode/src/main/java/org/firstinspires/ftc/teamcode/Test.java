package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;


/**The code pertaining to the IMU may need to be changed if
 * the controller hub was shipped after December 1, 2021**/

@Disabled
@Autonomous(group="HarshAutoTesting", name="AutoTesting")
public class Test extends LinearOpMode {
    /*This file was created and coded by Harshvardhan Nihalani*/

    //time
    private ElapsedTime runtime = new ElapsedTime();


    //REV 2m Distance sensor variables
//    private DistanceSensor sensorRange;



    //color sensor variables
//    NormalizedColorSensor colorSensor;
//    View relativeLayout;

//    final float[] hsvValues = new float[3];


    //IMU variables

    private int desiredHeading = 0;
    private boolean calib = false;
    public static String CALIB_FILE = "BNO055IMUCalibration.json";

    BNO055IMU imu;

    Orientation angles;

    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, angleCorrect;

    Acceleration gravity;



    //Dc motors

    private DcMotor Front_left = null;
    private DcMotor Back_left = null;
    private DcMotor Front_right = null;
    private DcMotor Back_right = null;
//    private DcMotor armMotor = null;
//    boolean armkill=false;


    //servo variables


//    static final double INCREMENT   = 0.02;     // amount to slew servo each CYCLE_MS cycle
//    static final int    CYCLE_MS    =   50;     // period of each cycle
//    static final double MAX_POS     =  0.3;     // Maximum rotational position for arm
//    static final double MIN_POS     =  0.62;     // Minimum rotational position for arm
//    static final double MAX_POS_claw     =  0.2;     // Maximum rotational position for claw
//    static final double MIN_POS_claw     =  0.0;  // Minimum rotational position for claw
//    double lowJunctionPOS=0.51;
//    double midJunctionPOS=0.45;
//    double highJunctionPOS=0.4;



    // servos
//    Servo   servo1arm; // servo motor for right arm
//    Servo   servo2arm; // servo motor for left arm
//    Servo   servo1claw; // servo motor for right claw
//    Servo   servo2claw; // servo motor for left claw
//    double  armPosition=MIN_POS;
//    double  clawPosition = MIN_POS_claw;



    //encoder variables
    static final double     COUNTS_PER_MOTOR_REV    = 5281.1*4 ;    // eg: GoBuilda Yellow Jacket Motor
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.77953 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.7;
    static final double     TURN_SPEED              = 0.5;










    @Override
    public void runOpMode() throws InterruptedException{
        set_config();
        imuTelemetry();



        waitForStart();


        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

//        float distToObj=getDistance();
//        getColor();

        doTelemetry();
        //Do stuff
        encoderDrive(DRIVE_SPEED,  48,  48,48,  48);  // S1: Forward 48 Inches with 5 Sec timeout
        doTelemetry();
        rotateDegrees(90,TURN_SPEED);
        doTelemetry();
        encoderDrive(DRIVE_SPEED, 24, 24,24, 24);  // S3: Reverse 24 Inches with 4 Sec timeout\
        doTelemetry();
        //update values
        Front_right.setPower(0);
        Front_left.setPower(0);
        Back_right.setPower(0);
        Back_left.setPower(0);


//        colorEnd();


    }





    public void set_config() {
        /*
        This function set configuration matching to driver controller configuration.
        Any addition or modification of hardware configuration can be done here

        This function utilizes method stacking

         */
        System.out.println("Config start");

        System.out.println("DCmotors start");
        DCmotorSetup();

        System.out.println("arm+claw servos start");
//        armSetup();

        //sensor setup
        System.out.println("internal IMU start");
        imuInitialization();

//        distanceSenorInit();
//
//        colorInit();

        System.out.println("Config end");
    }



    public void doTelemetry(){
        imuTelemetry();
//        colorTelem();
//        getDistance();
        telemetry.update();
    }

    public void DCmotorSetup(){
        Front_left  = hardwareMap.get(DcMotor.class, "FL");
        Back_left  = hardwareMap.get(DcMotor.class, "BL");
        Front_right = hardwareMap.get(DcMotor.class, "FR");
        Back_right = hardwareMap.get(DcMotor.class, "BR");


        Front_left.setDirection(DcMotor.Direction.REVERSE);
        Back_left.setDirection(DcMotor.Direction.REVERSE);
        Front_right.setDirection(DcMotor.Direction.FORWARD);
        Back_right.setDirection(DcMotor.Direction.FORWARD);


        Front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }

//    public void armSetup(){
//        servo1claw = hardwareMap.get(Servo.class, "right_claw");
//        servo2claw = hardwareMap.get(Servo.class, "left_claw");
//        servo2claw.setDirection(Servo.Direction.REVERSE);
//
//        servo1arm = hardwareMap.get(Servo.class, "right_arm");
//        servo2arm = hardwareMap.get(Servo.class, "left_arm");
//        servo1arm.setDirection(Servo.Direction.REVERSE);
//    }



    public void runToPosition(){
        Front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void switchToEncoders(){
        Front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turnOffEncoder(){
        Front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    //moving robot


    public void moveRobotTime(int milliseconds, float power, boolean strafe){
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
    public void turnRobotTime(int milliseconds, float power){
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







    public void encoderDrive(double speed, double leftFront, double rightFront,double leftBack,double rightBack) {
        switchToEncoders();
        int newLFTarget;
        int newRFTarget;
        int newLBTarget;
        int newRBTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLFTarget = Front_left.getCurrentPosition() + (int) (leftFront * COUNTS_PER_INCH);
            newRFTarget = Front_right.getCurrentPosition() + (int) (rightFront * COUNTS_PER_INCH);
            newLBTarget = Front_left.getCurrentPosition() + (int) (leftBack * COUNTS_PER_INCH);
            newRBTarget = Front_right.getCurrentPosition() + (int) (rightBack * COUNTS_PER_INCH);
            Front_left.setTargetPosition(newLFTarget);
            Back_left.setTargetPosition(newLBTarget);
            Front_right.setTargetPosition(newRFTarget);
            Back_right.setTargetPosition(newRBTarget);


            // Turn On RUN_TO_POSITION
            runToPosition();
            // reset the timeout time and start motion.
            runtime.reset();
            Front_left.setPower(Math.abs(speed));
            Front_right.setPower(Math.abs(speed));
            Back_left.setPower(Math.abs(speed));
            Back_right.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (Front_left.isBusy() && Front_right.isBusy() && Back_left.isBusy() && Back_right.isBusy())) {

            }



            // Stop all motion;
            Front_left.setPower(0.0);
            Front_right.setPower(0.0);
            Back_left.setPower(0.0);
            Back_right.setPower(0.0);

            // Turn off RUN_TO_POSITION
            turnOffEncoder();

//            sleep(250);   // optional pause after each move.
        }
    }




    //arm and claw stuff


//    public void liftArmLow(){
//        armPosition = lowJunctionPOS;
//        liftArm();
//    }
//    public void liftArmMiddle(){
//        armPosition = midJunctionPOS;
//        liftArm();
//    }
//    public void liftArmHigh(){
//        armPosition = highJunctionPOS;
//        liftArm();
//    }
//    public void liftArmGround(){
//        // This function set lift servo position for ground.
//        // Loop introduced to reduce speed of arm going down to ground and avoid accident or slippage.
//        while(armPosition<=MIN_POS) {
//            armPosition +=0.05;
//            liftArm();
//            //sleep(10);
//        }
//    }
//    public void openClaw(){
//        // This function open claw fully and set max claw position
//        clawPosition=MAX_POS_claw;
//        moveClaw();
//    }
//    public void closeClaw(){
//        // This function close claw fully and set claw position to minimum
//        clawPosition=MIN_POS_claw;//becomes 1.05 (does not damage the servo when cone is picked up)
//        moveClaw();
//    }
//
//    // supportive functions
//    public void liftArm(){
//        // This function set position of lift arm servo motors if position are in range of MAX and MIN position
//        if(armPosition>=MAX_POS&&armPosition<=MIN_POS){
//            servo1arm.setPosition(armPosition);
//            servo2arm.setPosition(armPosition);
//        }
//    }
//    public void moveClaw(){
//        // setting up claw position
//        servo1claw.setPosition(clawPosition);
//        servo2claw.setPosition(clawPosition);
//    }




    //IMU stuff, do not touch

    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }




//    private void checkDirection() {
//        // The gain value determines how sensitive the correction is to direction changes.
//        // experiment with robot to get small smooth direction changes
//        // to stay on a straight line.
//        double correction, angle, gain = .10;
//
//        angle = getAngle();
//
//        if (angle == 0)
//            correction = 0;             // no adjustment.
//        else
//            correction = -angle;        // reverse sign of angle for correction.
//
//        correction *= gain;
//
//    }




    /**
     * -180 to +180 degrees supported
     * + is left - is right
     */
    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotateDegrees(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        Front_left.setPower(leftPower);
        Front_right.setPower(rightPower);
        Back_left.setPower(leftPower);
        Back_right.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        Front_left.setPower(0);
        Front_right.setPower(0);
        Back_left.setPower(0);
        Back_right.setPower(0);

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }



//    public void colorInit(){
//        float gain = 1;
//
//
//        // Get a reference to our sensor object. It's recommended to use NormalizedColorSensor over
//        // ColorSensor, because NormalizedColorSensor consistently gives values between 0 and 1, while
//        // the values you get from ColorSensor are dependent on the specific sensor you're using.
//        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
//        colorSensor.setGain(gain);
//        // If possible, turn the light on in the beginning (it might already be on anyway,
//        // we just make sure it is if we can).
//        if (colorSensor instanceof SwitchableLight) {
//            ((SwitchableLight)colorSensor).enableLight(true);
//        }
//    }
//
//
//
//    public void colorTelem(){
//        getColor();
//    }
//
//
//
//   public void getColor(){
//       // Get the normalized colors from the sensor
//       NormalizedRGBA colors = colorSensor.getNormalizedColors();
//
//       /* Use telemetry to display feedback on the driver station. We show the red, green, and blue
//        * normalized values from the sensor (in the range of 0 to 1), as well as the equivalent
//        * HSV (hue, saturation and value) values. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
//        * for an explanation of HSV color. */
//
//       // Update the hsvValues array by passing it to Color.colorToHSV()
//       Color.colorToHSV(colors.toColor(), hsvValues);
//       telemetry.addLine()
//               .addData("Red", "%.3f", colors.red)
//               .addData("Green", "%.3f", colors.green)
//               .addData("Blue", "%.3f", colors.blue);
//       telemetry.addLine()
//               .addData("Hue", "%.3f", hsvValues[0])
//               .addData("Saturation", "%.3f", hsvValues[1])
//               .addData("Value", "%.3f", hsvValues[2]);
//       telemetry.addData("Alpha", "%.3f", colors.alpha);
//   }
//
//
//
//    /* If this color sensor also has a distance sensor, display the measured distance.
//     * Note that the reported distance is only useful at very close range, and is impacted by
//     * ambient light and surface reflectivity. */
//
//     public void colorDistance() {
//         if (colorSensor instanceof DistanceSensor) {
//             telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
//         }
//
//         telemetry.update();
//     }
//
//    public void colorEnd(){
//        relativeLayout.post(new Runnable() {
//            public void run() {
//                relativeLayout.setBackgroundColor(Color.WHITE);
//            }
//        });
//    }
//
//
//    public void distanceSenorInit(){
//        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
//        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;
//    }
//
//    public float getDistance(){
//         float distanceToObj = (float) sensorRange.getDistance(DistanceUnit.INCH);
//         telemetry.addData("range", String.format("%.01f in", distanceToObj));
//         return distanceToObj;
//    }









    //IMU initialization, ABSOLUTELY DO NOT TOUCH
    public void imuInitialization(){
        initIMU2(hardwareMap,telemetry,true);

        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();
    }

    public void initImu(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void initIMU2(HardwareMap ahwMap, Telemetry t, boolean calibrate){
        telemetry = t;
        this.calib = calibrate;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = this.calib;
        parameters.loggingTag          = "IMU";

        if (!this.calib) {
            parameters.calibrationDataFile = CALIB_FILE;
        }
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        imu = (BNO055IMU) ahwMap.get("imu");
        if (imu != null){
            imu.initialize(parameters);
            telemetry.addData("Info", "Gyroscope initialized");
        }
        else{
            telemetry.addData("Error", "Gyroscope failed");
        }

    }

    void imuTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }


}


