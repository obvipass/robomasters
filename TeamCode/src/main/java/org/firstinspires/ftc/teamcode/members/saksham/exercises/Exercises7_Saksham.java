package org.firstinspires.ftc.teamcode.members.saksham.exercises;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FRLib.robot.Robot;
import org.firstinspires.ftc.teamcode.FRLib.subsystems.MecanumDrive;

/*
1. Add a method to the ProgrammingBoard that allows you to change the
ZeroPowerBehavior of the motor, and then add to your OpMode where
pressing gamepad1.a sets it to BRAKE and gamepad1.b sets it to FLOAT.
2. Make the joystick less sensitive in the middle without losing range by
bringing in the squareInputWithSign() method from section 5.2 into your
opMode and using it.
*/
// 1. already done in FRLib
// 2. simple math method
// @TeleOp
public class Exercises7_Saksham extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this, MecanumDrive.RobotName.KEVIN);
        waitForStart();
        double x = squareKeepSign(gamepad1.left_stick_x);
        double y = squareKeepSign(-gamepad1.left_stick_y);
        double yaw = squareKeepSign(gamepad1.right_stick_x);

        robot.drive.driveVector(y, x, yaw, 0.2);
    }

    private double squareKeepSign(double input) {
        return Math.pow(input, 2) * Math.signum(input);
    }
}
