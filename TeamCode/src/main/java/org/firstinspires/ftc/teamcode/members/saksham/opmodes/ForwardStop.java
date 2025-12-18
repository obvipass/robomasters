package org.firstinspires.ftc.teamcode.members.saksham.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FRLib.robot.Robot;
import org.firstinspires.ftc.teamcode.FRLib.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Logger;

@Autonomous
public class ForwardStop extends LinearOpMode {
    Logger logger = new Logger(Logger.LoggerMode.CRITICAL, telemetry);

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this, logger, MecanumDrive.RobotName.KEVIN);
        waitForStart();

        robot.drive.driveVector(1, 0 , 0, 0.2);
        while (opModeIsActive()) {
            if (robot.distanceSensor.getDistanceInches() < 15) {
                robot.drive.brake(20000);
                break;
            }
        }
    }
}
