package org.firstinspires.ftc.teamcode.members.vishruth;

import static org.firstinspires.ftc.teamcode.utils.Logger.LoggerMode.CRITICAL;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.FRLib.robot.Robot;
import org.firstinspires.ftc.teamcode.FRLib.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Logger;
@TeleOp
public class SevenPointSevenExersizes extends LinearOpMode {
    Logger logger = new Logger(telemetry);

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this,logger, MecanumDrive.RobotName.BOB);

        waitForStart();

        while (opModeIsActive()){

            if (gamepad1.a){

                switch (robot.drive.frontLeft.getRawMotor().getZeroPowerBehavior()){
                    case FLOAT : robot.drive.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    break;
                    case BRAKE : robot.drive.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                }

            }
            robot.drive.frontLeft.setPower(squareWithSign(gamepad1.left_stick_y));
            logger.logData(CRITICAL,"FrontLeftPower",robot.drive.frontLeft.getRawMotor().getPower());
            logger.logData(CRITICAL,"FrontLeft ZeroPowerBehavior",robot.drive.frontLeft.getRawMotor().getZeroPowerBehavior().toString());
            logger.update();
        }
    }

    public double squareWithSign(double input){
        return input * input * ( input < 0 ? -1  : 1  );

    }
}
