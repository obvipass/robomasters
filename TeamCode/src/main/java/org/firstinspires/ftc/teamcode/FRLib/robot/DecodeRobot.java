package org.firstinspires.ftc.teamcode.FRLib.robot;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FRLib.hardware.IMUW;
import org.firstinspires.ftc.teamcode.FRLib.hardware.LimelightW;
import org.firstinspires.ftc.teamcode.FRLib.hardware.PinpointW;
import org.firstinspires.ftc.teamcode.FRLib.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.FRLib.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Logger;

public class DecodeRobot {
    public Launcher launcher;
    public MecanumDrive drive;
    public IMUW imu;
    public PinpointW pinpoint;
    public LimelightW limelight;
    Logger logger;

    public DecodeRobot(LinearOpMode opMode,Logger logger) {
        this.logger = logger;
        imu = new IMUW(opMode.hardwareMap,"imu", RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
        pinpoint = new PinpointW(opMode.hardwareMap, logger);
        launcher = new Launcher(opMode);
        drive = new MecanumDrive(opMode, logger, MecanumDrive.RobotName.BOB,imu);
        limelight = new LimelightW(opMode.hardwareMap, 0, logger);
    }

}
