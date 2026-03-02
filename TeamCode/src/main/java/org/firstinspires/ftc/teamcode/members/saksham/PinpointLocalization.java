package org.firstinspires.ftc.teamcode.members.saksham;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FRLib.robot.DecodeRobot;
import org.firstinspires.ftc.teamcode.utils.Logger;

@TeleOp
public class PinpointLocalization extends LinearOpMode {
    DecodeRobot robot;
    Logger logger = new Logger(telemetry);
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new DecodeRobot(this, logger);
        waitForStart();

        while (opModeIsActive()) {
            robot.pinpoint.logPositon();
            robot.drive.driveVector(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, 0.5);
            logger.update();
        }
    }
}
