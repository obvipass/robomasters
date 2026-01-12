package org.firstinspires.ftc.teamcode.members.saksham.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FRLib.robot.Robot;
import org.firstinspires.ftc.teamcode.FRLib.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Logger;

@TeleOp
public class RobotDrive extends LinearOpMode {
    Logger logger = new Logger(telemetry);
    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, logger, MecanumDrive.RobotName.BOB);
        waitForStart();
        while (opModeIsActive()) {
            robot.drive.driveVector(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, 0.5);
        }
    }
}
