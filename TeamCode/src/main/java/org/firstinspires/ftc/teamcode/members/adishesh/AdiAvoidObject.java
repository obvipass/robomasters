package org.firstinspires.ftc.teamcode.members.adishesh;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Adi's Object Avoider")
public class AdiAvoidObject extends LinearOpMode {
    static final double COUNTS_PER_MOTOR_REV = 537.7;    // eg: TETRIX Motor Encoder
    static final double COUNTS_PER_DEGREE = 11.06; // 8400/360
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double POWER = 0.2;

    private DcMotor frontLeftDrive = null;
    private DcMotor rearLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor rearRightDrive = null;
    private DistanceSensor distanceSensor;
    private IMU imu;


    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_motor");
        rearLeftDrive = hardwareMap.get(DcMotor.class, "back_left_motor");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_motor");
        rearRightDrive = hardwareMap.get(DcMotor.class, "back_right_motor");

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance_sensor");

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        rearLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        rearRightDrive.setDirection(DcMotor.Direction.FORWARD);

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        waitForStart();

        int leftDist = 7 * 12;
        int rightDist = 7 * 12 ;
        double initialTargetCounts = leftDist * COUNTS_PER_INCH;

      boolean success =   move(leftDist , rightDist, POWER);
      if (!success) {
          int countsAtObstacle = frontLeftDrive.getCurrentPosition(); // counts at reaching obstacle
          double newTargetCounts = initialTargetCounts - countsAtObstacle;
          int lateralDist = moveRightUntilNoObstacleInSight();
          move(newTargetCounts, newTargetCounts, POWER);
          move(lateralDist, -lateralDist, POWER); // move left
      }

        sleep(1000);

    }

    private int moveRightUntilNoObstacleInSight() {
        int count = 0;
        while (distanceSensor.getDistance(DistanceUnit.INCH) <= 20 ) {
            move(-1,1, POWER); // Move inch by inch until no sight of obstacle.
            count += 1;
        }
        move(-10,10, POWER); // Buffer for left arm of the robot.
        count += 10;
        return count;
    }

    private boolean move(int leftDist, int rightDist, double power) {
        return move(leftDist * COUNTS_PER_INCH,rightDist * COUNTS_PER_INCH, power );
    }


    private boolean move(double leftDistCounts, double rightDistCounts, double power) {
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int newFrontLeftPosition = frontLeftDrive.getCurrentPosition() + (int) leftDistCounts;
        int newFrontRightPosition = frontRightDrive.getCurrentPosition() + (int) rightDistCounts;
        int newRearLeftPosition = rearLeftDrive.getCurrentPosition() + (int) leftDistCounts;
        int newRearRightPosition = rearLeftDrive.getCurrentPosition() + (int) rightDistCounts;


        frontLeftDrive.setTargetPosition(newFrontLeftPosition);
        frontRightDrive.setTargetPosition(newFrontRightPosition);
        rearLeftDrive.setTargetPosition(newRearLeftPosition);
        rearRightDrive.setTargetPosition(newRearRightPosition);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        rearLeftDrive.setPower(power);
        rearRightDrive.setPower(power);


        while (opModeIsActive() && frontLeftDrive.isBusy() && frontRightDrive.isBusy() &&
                rearLeftDrive.isBusy() && rearRightDrive.isBusy() && distanceSensor.getDistance(DistanceUnit.INCH) > 20) {
            removeYaw(power);
        }

        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        rearLeftDrive.setPower(0);
        rearRightDrive.setPower(0);


        telemetry.addData("Moved to ", "%7d %7d %7d %7d", newFrontLeftPosition, newFrontRightPosition, newRearLeftPosition, newRearRightPosition);
        telemetry.update();

        if (distanceSensor.getDistance(DistanceUnit.INCH) <= 20)  {
            return false;
        }
        return true;

    }

    private void removeYaw(double power) {
        while (opModeIsActive() && frontLeftDrive.isBusy() && frontRightDrive.isBusy() && rearLeftDrive.isBusy() && rearRightDrive.isBusy()) {
            int targetYaw = 0;
            double currentYaw = imu.getRobotYawPitchRollAngles().getYaw();
            double error = targetYaw - currentYaw;

            double errorGain = 0.01 * error; // Apply correction gradually

            double leftPower = power - errorGain;
            double rightPower = power + errorGain;
            double max = Math.max(leftPower, rightPower);
            if (max >= 1.0) {
                leftPower = 0.2 * leftPower;
                rightPower = 0.2 * rightPower;
            }
            frontLeftDrive.setPower(leftPower);
            rearLeftDrive.setPower(leftPower);
            frontRightDrive.setPower(rightPower);
            rearRightDrive.setPower(rightPower);
        }
    }

}



