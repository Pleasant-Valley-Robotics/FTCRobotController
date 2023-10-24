package org.firstinspires.ftc.teamcode.drive;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "Red_Backstage_45")
public class Auto1 extends LinearOpMode{
    DcMotor FLDrive = null; // standard motor declarations
    DcMotor FRDrive = null;
    DcMotor BLDrive = null;
    DcMotor BRDrive = null;
    IMU imu = null;

    public void runOpMode() throws InterruptedException {

        double desiredHeading = 0;

        FLDrive = hardwareMap.get(DcMotor.class, "FLDrive");
        FRDrive = hardwareMap.get(DcMotor.class, "FRDrive");
        BLDrive = hardwareMap.get(DcMotor.class, "BLDrive");
        BRDrive = hardwareMap.get(DcMotor.class, "BRDrive");

        FLDrive.setDirection(DcMotor.Direction.REVERSE);
        BLDrive.setDirection(DcMotor.Direction.REVERSE);
        FRDrive.setDirection(DcMotor.Direction.REVERSE);
        BRDrive.setDirection(DcMotor.Direction.REVERSE);

        FLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
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

        //TODO: WRITE YOUR AUTO CODE HERE!
        driveStraight(10, 0.7);
        turnToAngle(90.0);
        driveStraight(20, 0.4);
        turnToAngle(180.0);

    }

    void driveStraight(double inches, double power){
        double i = inches;
        int ticks = inchesToTicks(i);
        double pwr = power;

        int encoderError = 10;

        motorsToPosition(ticks,  pwr);
        stopMove();

    }

    void motorsToPosition(int encoders, double power) {
        FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FLDrive.setTargetPosition(encoders);
        FRDrive.setTargetPosition(encoders);
        BLDrive.setTargetPosition(encoders);
        BRDrive.setTargetPosition(encoders);

        FLDrive.setPower(power);
        FRDrive.setPower(power);
        BLDrive.setPower(power);
        BRDrive.setPower(power);

        FLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (motorsActive() && opModeIsActive()) {
            telemetry.addData("FRD", FRDrive.getCurrentPosition());
            telemetry.addData("FRDP", FRDrive.getPower());
            telemetry.addData("FLD", FLDrive.getCurrentPosition());
            telemetry.update();
            try {
                Thread.sleep(10);
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
        }
    }

    boolean motorsActive() {
        return FLDrive.isBusy() || FRDrive.isBusy() || BLDrive.isBusy() || BRDrive.isBusy();
    }

    public boolean isBusy() {
        return FLDrive.isBusy() || FRDrive.isBusy() || BLDrive.isBusy() || BRDrive.isBusy();
    }

    int inchesToTicks(double inches){
        return (int) inches*134;
    }

    //This turns the robot. Don't question it Will did it.
    void turnToAngle(double angle) {
        double botHeadingDeg = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double rotate = botHeadingDeg - angle; // algorithm for automatic turning
        rotate += 540;
        rotate = (rotate % 360) - 180;

        double rx = rotate / 70;
        FLDrive.setPower(rx);
        BLDrive.setPower(rx);
        BLDrive.setPower(-rx);
        BRDrive.setPower(-rx);

        while (Math.abs(botHeadingDeg - angle) > 10) {// 10 degree margin
            botHeadingDeg = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }
    }

    public void stopMove() {
        FLDrive.setPower(0);
        FRDrive.setPower(0);
        BLDrive.setPower(0);
        BRDrive.setPower(0);
    }

}