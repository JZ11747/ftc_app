package org.firstinspires.ftc.teamcode;//package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by femukund on 10/22/2017.
 */

public class Robot
{
    //    DcMotor motorTest;
    HardwareMap hwMap;
    LinearOpMode opMode;

    // Robot wheels
    DcMotor rightFrontMotor;
    DcMotor leftFrontMotor;
    DcMotor rightBackMotor;
    DcMotor leftBackMotor;

    // Lift & Claw
    DcMotor rightArmMotor;
    DcMotor leftArmMotor;
    Servo frontArmServo;
    Servo backArmServo;
    DcMotor liftMotor;
    Servo clawLeftServo;
    Servo clawRightServo;
    TouchSensor liftTouchSensor;

    // Jewel Knocker
    Servo jewelServo;
    private ColorSensor colorSensor;

    public void init(HardwareMap ProtohwMap, LinearOpMode linearOpMode)
    {
        hwMap = ProtohwMap;
        opMode = linearOpMode;

        leftFrontMotor = hwMap.dcMotor.get("leftFrontMotor");
        leftBackMotor = hwMap.dcMotor.get("leftBackMotor");
        rightFrontMotor = hwMap.dcMotor.get("rightFrontMotor");
        rightBackMotor = hwMap.dcMotor.get("rightBackMotor");
        rightArmMotor = hwMap.dcMotor.get("rightArmMotor");
        leftArmMotor = hwMap.dcMotor.get("leftArmMotor");
        frontArmServo = hwMap.servo.get("frontArmServo");
        backArmServo = hwMap.servo.get("backArmServo");

        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //liftMotor = hwMap.dcMotor.get("liftMotor");
        //clawLeftServo = hwMap.servo.get("clawLeftServo");
        //clawRightServo = hwMap.servo.get("clawRightServo");
        //liftTouchSensor= hwMap.touchSensor.get("liftTouchSensor");

        //jewelServo = hwMap.servo.get("jewelServo");
        //colorSensor = hwMap.get(ColorSensor.class, "colorSensor");

    }

    public boolean isJewelRed()
    {
        boolean isJewelRed = true;
        colorSensor.enableLed(true);
        int redColor = colorSensor.red();
        int blueColor = colorSensor.blue();
        opMode.telemetry.addData("Red Color Value:", colorSensor.red());
        opMode.telemetry.addData("Blue Color Value:", colorSensor.blue());
        opMode.telemetry.update();
        if (redColor > blueColor)
        {
            isJewelRed = true;
        }
        else
        {
            isJewelRed = false;
        }

        //WaitMillis(2000);
        colorSensor.enableLed(false);
        return isJewelRed;
    }

    // Lower Jewel Servo
    public void lowerJewelServo()
    {
        jewelServo.setPosition(0.16);
        opMode.telemetry.addData("Servo Position 2:", jewelServo.getPosition());
        opMode.telemetry.update();
        WaitMillis(2000);
    }

    // Raise Jewel Servo
    public void raiseJewelServo()
    {
        jewelServo.setPosition(0.85);
        opMode.telemetry.addData("Servo Position 3:", jewelServo.getPosition());
        opMode.telemetry.update();
        WaitMillis(4000);
    }

    public void WaitMillis (long millis)
    {
        try{
            Thread.sleep(millis);
        }
        catch (InterruptedException ex)
        {
            Thread.currentThread().interrupt();
        }
    }

    public void Drive(double leftFront, double rightFront, double leftBack, double rightBack, long millis)
    {
        leftFrontMotor.setPower(leftFront);
        rightFrontMotor.setPower(rightFront);
        leftBackMotor.setPower(leftBack);
        rightBackMotor.setPower(rightBack);
        WaitMillis(millis);
    }

    public void DriveForward(double power, long millis)
    {
        leftFrontMotor.setPower(-power);
        rightFrontMotor.setPower(-power);
        leftBackMotor.setPower(-power);
        rightBackMotor.setPower(-power);
        WaitMillis(millis);
    }

    public void DriveBackwards(double power, long millis)
    {
        leftFrontMotor.setPower(power);
        rightFrontMotor.setPower(power);
        leftBackMotor.setPower(power);
        rightBackMotor.setPower(power);
        WaitMillis(millis);
    }

    public void DriveLeft(double power, long millis)
    {
        leftFrontMotor.setPower(-power);
        leftBackMotor.setPower(power);
        rightFrontMotor.setPower(power);
        rightBackMotor.setPower(-power);
        WaitMillis(millis);
    }

    public void DriveRight(double power, long millis)
    {
        leftFrontMotor.setPower(power);
        leftBackMotor.setPower(-power);
        rightFrontMotor.setPower(-power);
        rightBackMotor.setPower(power);
        WaitMillis(millis);
    }

    public void Brake(long millis)
    {
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        WaitMillis(millis);
    }

    public void holdGlyph()
    {
        clawLeftServo.setPosition(0.6);
        clawRightServo.setPosition(0.4);
    }

    public void raiseLift()
    {
        liftMotor.setPower(-0.2);
        WaitMillis(2000);
        liftMotor.setPower(0);
    }

    public void lowerLift()
    {
        liftMotor.setPower(0.2);
        WaitMillis(1200);
        liftMotor.setPower(0);
    }

    public void openClaw()
    {
        clawLeftServo.setPosition(0);
        clawRightServo.setPosition(1);
        WaitMillis(1200);
    }

    public void rotateArm(long waitTime, double power)
    {
        rightArmMotor.setPower(power);
        leftArmMotor.setPower(-power);
        WaitMillis(waitTime);
        rightArmMotor.setPower(-power);
        leftArmMotor.setPower(power);
        rightArmMotor.setPower(0);
        leftArmMotor.setPower(0);
    }

}
