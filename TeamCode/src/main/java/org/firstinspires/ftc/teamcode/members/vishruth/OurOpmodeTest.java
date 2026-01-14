package org.firstinspires.ftc.teamcode.members.vishruth;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FRLib.robot.Robot;
import org.firstinspires.ftc.teamcode.FRLib.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Logger;
@TeleOp
public class OurOpmodeTest extends OurOpmode{
    @Override
    protected void Loop() {
        robot.drive.driveVectorField(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x,0.2,robot.imu);
    }

    @Override
    protected void initialize() {
        logger = new Logger(telemetry);
        robot = new Robot(this,this.logger, MecanumDrive.RobotName.BOB);
        logger.logData(Logger.LoggerMode.STATUS,"Robot Init","");
        logger.update();

    }
}
