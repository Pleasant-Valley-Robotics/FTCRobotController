package org.firstinspires.ftc.teamcode.drive;

import static java.lang.Math.signum;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "TestingAuto", group = "Iterative Opmode")
public class TestingTele extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor FLDrive = null; // standard motor declarations
        DcMotor FRDrive = null;
        DcMotor BLDrive = null;
        DcMotor BRDrive = null;
        DcMotor liftDrive = null;
        DcMotor liftJoint = null;
        DcMotor leftActuator = null;
        DcMotor rightActuator = null;

        //Write numerical variables here
        double desiredHeading = 0;
        FLDrive = hardwareMap.get(DcMotor.class, "FLDrive");
        FRDrive = hardwareMap.get(DcMotor.class, "FRDrive");
        BLDrive = hardwareMap.get(DcMotor.class, "BLDrive");
        BRDrive = hardwareMap.get(DcMotor.class, "BRDrive");
        liftDrive = hardwareMap.get(DcMotor.class, "liftDrive");
        liftJoint = hardwareMap.get(DcMotor.class, "liftJoint");
        leftActuator = hardwareMap.get(DcMotor.class, "leftActuator");
        rightActuator = hardwareMap.get(DcMotor.class, "rightActuator");

        FLDrive.setDirection(DcMotor.Direction.FORWARD);
        BLDrive.setDirection(DcMotor.Direction.REVERSE);
        FRDrive.setDirection(DcMotor.Direction.FORWARD);
        BRDrive.setDirection(DcMotor.Direction.REVERSE);
        liftDrive.setDirection(DcMotor.Direction.FORWARD);
        liftJoint.setDirection(DcMotor.Direction.FORWARD);
        leftActuator.setDirection(DcMotor.Direction.FORWARD);
        rightActuator.setDirection(DcMotor.Direction.FORWARD);

        FLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftJoint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftActuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightActuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        imu.resetYaw();

        waitForStart();
        double yHeading = 0;
        double xHeading = -90;
        double bHeading = 90;
        double aHeading = 180;


        if (isStopRequested()) return;

        while (opModeIsActive()) {
            //TODO: Finish write tele-op

            double speedMultiplier;

            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.y) {  //automatic turning commands
                desiredHeading = yHeading;
            }
            if (gamepad1.x) {
                desiredHeading = xHeading;
            }
            if (gamepad1.b) {
                desiredHeading = bHeading;
            }
            if (gamepad1.a) {
                desiredHeading = aHeading;
            }

            boolean slowMode = gamepad1.right_bumper;
            if (slowMode) {
                speedMultiplier = .5;
            } else {
                speedMultiplier = 1.0;
            }

            //TODO: WRITE MORE CODE HERE TO MAKE MOTORS MOVE
 //THIS CODE IS FOR USING THE LIFT WITH BUTTONS. TESTING PURPOSES ONLY!
            if(gamepad2.x && (!gamepad2.y)){
               liftDrive.setPower(0.7);
            }else if(gamepad2.y && !(gamepad2.x)){
                liftDrive.setPower(-0.7);
            }else{
                liftDrive.setPower(0);
            }


            //THIS CODE IS FOR USING THE LIFT WITH JOYSICKS LIKE A NORMAL PERSON
             //For digital
            //boolean liftJoysick = ((gamepad2.left_stick_y)>0.05) || ((gamepad2.left_stick_y)< -0.05);
            
            /*if (liftJoystick){
                liftDrive.setPower(0.7);
            }
            */

             //For analog
            double liftJoystick = gamepad2.left_stick_y;
            if(liftJoystick>1){
                liftJoystick = 1.0;
            }
            
            if(liftJoystick<-1){
                liftJoystick = -1.0;
            }

            if(liftJoystick>0.05 || liftJoystick<-0.05){
                liftDrive.setPower(liftJoystick);
            } else{
                liftDrive.setPower(0);
            }
            //End of liftDrive code
            
            //Start of Actuator

            boolean actuatorMoveUp = gamepad2.dpad_up;
            boolean actuatorMoveDown = gamepad2.dpad_down;

            if(actuatorMoveUp && !actuatorMoveDown){
                leftActuator.setPower(0.7);
                rightActuator.setPower(0.7);
            } else if(actuatorMoveDown && !actuatorMoveUp){
                leftActuator.setPower(-0.7);
                rightActuator.setPower(-0.7);
            }

            //digital
            /*
            boolean jointMove ((gamepad2.right_stick_y)>0.05) || ((gamepad2.right_stick_y)< -0.05);
            
            if (jointMove){
                liftJoint.setPower(0.7);
            }

             */
            


            //analog
            double jointMove = gamepad2.right_stick_y;
            if(jointMove>1.0){
                jointMove = 1.0;
            }

            if(jointMove>0.05 || jointMove<-0.05){
                liftJoint.setPower(jointMove);
            } else{
                liftJoint.setPower(0);
            }

            double botHeadingDeg = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double rotate = botHeadingDeg - desiredHeading; // algorithm for automatic turning
            rotate += 540;
            rotate = (rotate % 360) - 180;
            rx += rotate/-70;

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); // bot heading for field centric
            // Rotate the movement direction counter to the bot's rotation
            // Changes x and y from robot centric drive to field-centric
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX - rx) / denominator; // standard mecanum wheel formulas
            double backLeftPower = (rotY - rotX - rx) / denominator;
            double frontRightPower = (rotY - rotX + rx) / denominator;
            double backRightPower = (rotY + rotX + rx) / denominator;

            FLDrive.setPower(frontLeftPower * speedMultiplier); // set power to wheels
            BLDrive.setPower(backLeftPower * speedMultiplier);
            FRDrive.setPower(frontRightPower * speedMultiplier);
            BRDrive.setPower(backRightPower * speedMultiplier);

            if(((gamepad1.right_trigger)>.05) && ((gamepad1.left_trigger))>.05){
                imu.resetYaw();
            }

        }

    }
}
