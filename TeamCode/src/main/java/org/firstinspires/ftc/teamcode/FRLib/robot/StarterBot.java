package org.firstinspires.ftc.teamcode.FRLib.robot;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.FRLib.hardware.Distance2mW;
import org.firstinspires.ftc.teamcode.FRLib.hardware.IMUW;
import org.firstinspires.ftc.teamcode.FRLib.hardware.TouchSensorW;
import org.firstinspires.ftc.teamcode.FRLib.subsystems.MecanumDrive;

public class StarterBot {
    public final DcMotor leftDrive;
    public final DcMotor rightDrive;
    public final DcMotorEx launcher;
    public final CRServo leftFeeder;
    public final CRServo rightFeeder;
    public final IMUW imu;
    public double MAX_CAPABLE_VELOCITY = (312 * (537.7 / 60)); // 2796.04

    public StarterBot(LinearOpMode opMode, Logger logger) {
        HardwareMap hardwareMap = opMode.hardwareMap;
        if (hardwareMap == null) {
            throw new IllegalStateException("hardwareMap accessed before init()");
        }

        imu = new IMUW(hardwareMap, "imu",
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT);

        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        launcher = hardwareMap.get(DcMotorEx.class,"launcher");

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300,0,0,10));

        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE); // reverse left feeder bcz it's on opposite side
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
    }

    public StarterBot(LinearOpMode opMode) {
        this(opMode, new Logger(Logger.LoggerMode.CRITICAL, opMode.telemetry));
    }
}
