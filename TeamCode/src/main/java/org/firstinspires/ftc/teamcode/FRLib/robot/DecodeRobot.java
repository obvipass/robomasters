package org.firstinspires.ftc.teamcode.FRLib.robot;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FRLib.hardware.IMUW;
import org.firstinspires.ftc.teamcode.FRLib.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.FRLib.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Logger;

public class DecodeRobot {
    public Launcher launcher;
    public MecanumDrive drive;
    public IMUW imu;
    Logger logger;

    public DecodeRobot(LinearOpMode opMode,Logger logger1) {
        logger = logger1;
        imu = new IMUW(opMode.hardwareMap,"imu", RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
        launcher = new Launcher(opMode);
        drive = new MecanumDrive(opMode,logger, MecanumDrive.RobotName.BOB,imu);
    }

}
