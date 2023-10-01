package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Disabled
@TeleOp(name="use this teleop", group="notTest")
public class robotTeleOp extends LinearOpMode {

    // create motor variables
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor front_left_dcMotor = null;
    private DcMotor back_left_dcMotor = null;
    private DcMotor front_right_dcMotor = null;
    private DcMotor back_right_dcMotor = null;


    //create servo variables

    //in claw
    Servo inClaw;
    double inClawPos=0.0; // change this value to initial value
    final double inClawOpen=0.0; // change this value
    final double inClawClosed=0.0; // change this value
    final double inClawClosedPixel=0.0; // change this value

    Servo inClawHeightAdjuster;
    int dpadPressCounter=0;
    double inClawHeightAdjusterPos=0.0; // change this value to initial value
    final double inHeightNormal=0.0; // change this value (if only 2 pixels are in stack)
    final double inHeightStack5=0.0; // change this value
    final double inHeightStack4=0.0; // change this value
    final double inHeightStack3=0.0; // change this value
    final double inHeightClawTransferPos=0.0; // change this value

    Servo inClawWrist;
    double inClawWristPos=0.0; // change this value to initial value
    final double inWristNormal=0.0; // change this value (if only 2 pixels are in stack)
    final double inWristHeight5=0.0; // change this value
    final double inWristHeight4=0.0; // change this value
    final double inWristHeight3=0.0; // change this value
    final double inWristClawTransferPos=0.0; // change this value

    boolean inTransferInProg=false;

    //out claw
    Servo outClaw;
    double outClawPos=0.0; // change this value to initial value
    final double outClawOpen=0.0; // change this value
    final double outClawClosed=0.0; // change this value
    final double outClawPixel=0.0;//change this value

    Servo outDistanceAdjuster;
    double outDistanceAdjusterPos=0.0; // change this value to initial value
    final double outDistanceTransfer=0.0; // change this value
    final double outDistancePlace=0.0; // change this value

    Servo outWrist;
    double outWristPos=0.0; // change this value to initial value
    final double outWristTransfer=0.0; // change this value
    final double outWristPlace=0.0; // change this value

    boolean outTransferInProg =false;


    Servo transfer;
    double transferPos=0.0; //change this value to initial value
    final double transferOut=0.0; //change this value
    final double transferStay=0.0; //change this value


    @Override
    public void runOpMode() {
        set_config();

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        moveServos();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Send calculated power to wheels
            front_left_dcMotor.setPower(leftFrontPower);
            front_right_dcMotor.setPower(rightFrontPower);
            back_left_dcMotor.setPower(leftBackPower);
            back_right_dcMotor.setPower(rightBackPower);


            getToIntakePos();

            intakeAndTransferToTransfer();

            outReturnToTransfer();
            openOutClaw();
            placePixel();
            if(gamepad2.a){
                transferOut();
            }


            moveServos();

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }


    public void getToIntakePos(){
        if(gamepad2.dpad_down){
            dpadPressCounter++;
        }
       else if(gamepad2.dpad_right){
            dpadPressCounter--;
       }

        if(dpadPressCounter==1&&!inTransferInProg){
            inClawHeightAdjusterPos=inHeightStack5;
            inClawWristPos=inWristHeight5;
        }
        else if(dpadPressCounter==2&&!inTransferInProg){
            inClawHeightAdjusterPos=inHeightStack4;
            inClawWristPos=inWristHeight4;
        }
        else if(dpadPressCounter==3&&!inTransferInProg){
            inClawHeightAdjusterPos=inHeightStack3;
            inClawWristPos=inWristHeight3;
        }
        else if(dpadPressCounter==4&&!inTransferInProg){
            inClawHeightAdjusterPos=inHeightNormal;
            inClawWristPos=inWristNormal;
        }
        else if(dpadPressCounter>4){
            dpadPressCounter%=4;
            dpadPressCounter++;
        }

        if(gamepad2.x&&!inTransferInProg){
            dpadPressCounter=4;
            inClawHeightAdjusterPos=inHeightNormal;
        }
    }
    public void intakeAndTransferToTransfer(){
        if(gamepad2.right_bumper&&inClawHeightAdjuster.getPosition()==inClawHeightAdjusterPos&&inClawWristPos==inClawWrist.getPosition()) {
            inClawPos = inClawClosed;
            moveServos();
            //sleep(100);
            inTransferInProg=true;
            dpadPressCounter=0;

        }
        if(inClaw.getPosition()==inClawClosedPixel&&inTransferInProg)
            inClawHeightAdjusterPos=inHeightClawTransferPos;
        if(inClawHeightAdjuster.getPosition()==inClawHeightAdjusterPos&&inTransferInProg) {
            //sleep(200);
            inClawPos = inClawOpen;
            inTransferInProg=false;
        }
    }

    public void transferOut(){
        ;
    }
    public void placePixel(){
        if(outClawPos==outClawPixel&&outClawPos==outClaw.getPosition()){
            //sleep(50);
            outWristPos=outWristPlace;
            outDistanceAdjusterPos=outDistancePlace;
            outTransferInProg=true;
        }


    }
    public void openOutClaw(){
        if(outDistanceAdjusterPos==outDistanceAdjuster.getPosition()&&outWristPos==outWrist.getPosition()&&gamepad2.left_bumper&& outTransferInProg){
            outClawPos=outClawOpen;
            outTransferInProg =false;
        }
    }
    public void outReturnToTransfer(){
        if(!outTransferInProg &&outClaw.getPosition()==outClawOpen){
            outWristPos=outWristTransfer;
            outDistanceAdjusterPos=outDistanceTransfer;
        }
    }


    public void moveServos(){
        inClaw.setPosition(inClawPos);
        inClawHeightAdjuster.setPosition(inClawHeightAdjusterPos);
        inClawWrist.setPosition(inClawWristPos);
        outClaw.setPosition(outClawPos);
        outDistanceAdjuster.setPosition(outDistanceAdjusterPos);
        outWrist.setPosition(outWristPos);
        transfer.setPosition(transferPos);
    }












    public void set_config(){
        dcMotorConfig();
        servoConfig();
    }

    public void dcMotorConfig(){
        front_left_dcMotor  = hardwareMap.get(DcMotor.class, "left_front_drive");//change this name
        back_left_dcMotor = hardwareMap.get(DcMotor.class, "left_back_drive");//change this name
        front_right_dcMotor = hardwareMap.get(DcMotor.class, "right_front_drive");//change this name
        back_right_dcMotor = hardwareMap.get(DcMotor.class, "right_back_drive");//change this name

        front_left_dcMotor.setDirection(DcMotor.Direction.REVERSE);
        back_left_dcMotor.setDirection(DcMotor.Direction.REVERSE);
        front_right_dcMotor.setDirection(DcMotor.Direction.FORWARD);
        back_right_dcMotor.setDirection(DcMotor.Direction.FORWARD);
    }
    public void servoConfig(){
        //assumes 7 servos
        inClaw=hardwareMap.get(Servo.class, "inClaw"); //change this name
        inClawHeightAdjuster=hardwareMap.get(Servo.class, "inClawHeightAdjuster"); //change this name
        inClawWrist=hardwareMap.get(Servo.class, "inClawWrist"); //change this name
        outClaw=hardwareMap.get(Servo.class, "outClaw"); //change this name
        outDistanceAdjuster=hardwareMap.get(Servo.class, "outDistanceAdjuster"); //change this name
        outWrist=hardwareMap.get(Servo.class, "outWrist"); //change this name
        transfer=hardwareMap.get(Servo.class, "transfer"); //change this name
    }






}
