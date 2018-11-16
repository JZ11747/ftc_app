//package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by femukund on 11/5/2017.
 */

@Autonomous(name="ExtremeBotAuto")
public class ExtremeBotAuto extends LinearOpMode
{
    Robot robot = new Robot();

    @Override
    public void runOpMode()
    {
        double drivePower = 0.2;

        robot.init(hardwareMap, this);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        telemetry.addData("Say", "All systems are go!");
        //GO!!!

        // Hold glyph
        //robot.openClaw();
        //robot.jewelServo.setPosition(0.85);

        waitForStart();
        //robot.holdGlyph();
        //robot.raiseLift();

        // Lower jewel servo

        //robot.lowerJewelServo();
        //telemetry.addData("Say", "Servo is lowered.");
        //telemetry.update();

        // Sense the color of the jewel
       //boolean isJewelRed = robot.isJewelRed();
        //telemetry.addData("Is Jewel Red:", isJewelRed);

        // Knock off jewel
        long driveForwardTime = 2000;
        long driveBackwardTime = 2000;
        long driveLeftTime = 2000;
        long driveRightTime = 2000;
        /*if (isJewelRed)
        {
            robot.DriveForward(drivePower, 200);
            robot.Brake(500);
            driveForwardTime2 = 1050;
        }
        else
        {
            robot.DriveBackwards(drivePower, 200);
            robot.Brake(500);
            driveForwardTime2 = 1250;
        }

        // Raise jewel servo
//        robot.WaitMillis(3000);
        robot.raiseJewelServo();
        telemetry.addData("Say", "Servo is raised.");
        telemetry.update();
        */

        // Drive to Cryptobox
        robot.DriveForward(drivePower, driveForwardTime);
        robot.Brake(2000);
        robot.DriveBackwards(drivePower, driveBackwardTime);
        robot.Brake(4000);
        robot.rotateArm(2000, 0.3);
        robot.Brake(4000);
        robot.DriveLeft(0.35, driveLeftTime);
        robot.Brake(4000);
        robot.DriveRight(0.35, driveRightTime);
        robot.Brake(4000);
        //robot.TankRight(drivePower, 2150);
        //robot.Brake(500);

        // Place the glyph
        //robot.lowerLift();
        //robot.openClaw();
        //robot.DriveForward(drivePower, 600);
        //robot.DriveBackwards(drivePower, 300);
        // Park in the triangle

        telemetry.addData("Say", "I am done.");
        telemetry.update();
    }
}
