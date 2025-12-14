package org.firstinspires.ftc.teamcode.members.shreyank;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@Autonomous(name="Obstacle Avoidance", group = "Shreyank")
public class DetectObstacles extends LinearOpMode {

    private  IMU imu ;
    private DistanceSensor sensorDistance;

    Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;
    // Declare OpMode members for each of the 4 motors.
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;

    final double COUNTS_PER_REVOLUTION = 537.7;
    final double WHEEL_DIAMETER_INCHES = 4.09;
    final double countsPerInch = COUNTS_PER_REVOLUTION / (WHEEL_DIAMETER_INCHES * Math.PI);
    //final double countsPerDegree = COUNTS_PER_REVOLUTION/45;
    final double countsPerDegree = 11.06;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables.
        // These motor names must correspond to the names configured in the robot configuration file.
        frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRightMotor = hardwareMap.get(DcMotor.class, "back_right_motor");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "distance_sensor");

        // Set motor directions.
        // You may need to reverse motors depending on your physical setup.
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY).
        waitForStart();

        if (opModeIsActive()) {

            moveForward(84);


            telemetry.addData("Status", "Done");
            telemetry.update();
        }
    }

    private void moveForward(double distance) {
        int countsToMove= (int)( distance * countsPerInch);
        applyEncoderObstacleAvoidance(countsToMove, countsToMove,countsToMove, countsToMove,0.1);
    }

    private void moveBackward(double distance) {
        int countsToMove= (int)( distance * countsPerInch*-1);
        applyEncoder(countsToMove, countsToMove,countsToMove, countsToMove,0.2);
    }

    private void rotate(int degrees) {
        int countsToMove = (int) countsPerDegree*degrees;
        applyEncoder(countsToMove, -countsToMove, countsToMove, -countsToMove, 0.2);
    }
    private void applyEncoder(int frontLeftTarget, int frontRightTarget, int rearLeftTarget, int rearRightTarget, double power) {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Turn the motor back on, required if you use STOP_AND_RESET_ENCODER
        /*frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        */

        frontLeftMotor.setTargetPosition(frontLeftTarget);
        frontRightMotor.setTargetPosition(frontRightTarget);
        backLeftMotor.setTargetPosition(rearLeftTarget);
        backRightMotor.setTargetPosition(rearRightTarget);


        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);

        //imu.resetYaw();


        imu = hardwareMap.get(IMU.class, "imu");
        imu.resetYaw();

        while(frontLeftMotor.isBusy() && frontRightMotor.isBusy() &&
                backLeftMotor.isBusy() && backRightMotor.isBusy() && opModeIsActive()) {
            //Loop body can be empty
            // Get the current heading (yaw angle)

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double currentHeading = orientation.getYaw(AngleUnit.DEGREES);

            // Calculate the deviation (error) from the target heading
            double targetHeading = 0;
            double headingError = targetHeading - currentHeading;

            // Apply proportional correction: larger error means larger correction
            double HEADING_GAIN = 0.01;
            headingError = headingError*HEADING_GAIN;


            // Adjust motor powers based on the correction
            double frontLeftPower = power + headingError;
            double frontRightPower = power - headingError;
            double rearLeftPower = power - headingError;
            double rearRightPower = power + headingError;

            // Clip motor powers to stay within the valid range (-0.3 to 0.3)
            frontLeftPower = Range.clip(frontLeftPower, -0.3, 0.3);
            frontRightPower = Range.clip(frontRightPower, -0.3, 0.3);
            rearLeftPower = Range.clip(rearLeftPower, -0.3, 0.3);
            rearRightPower = Range.clip(rearRightPower, -0.3, 0.3);

            // Set motor powers
            frontLeftMotor.setPower(frontLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backLeftMotor.setPower(rearLeftPower);
            backRightMotor.setPower(rearRightPower);

            // Add telemetry for monitoring
            telemetry.addData("Target Heading", targetHeading);
            telemetry.addData("Current Heading", currentHeading);
            telemetry.addData("Correction", headingError);
            telemetry.addData("Front Left Power", frontLeftPower);
            telemetry.addData("Front Right Power", frontRightPower);
            telemetry.addData("Rear Left Power", rearLeftPower);
            telemetry.addData("Rear Right Power", rearRightPower);

            telemetry.update();
        }

        // Stop all motors after the timeout
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    private void applyEncoderObstacleAvoidance(int frontLeftTarget, int frontRightTarget, int rearLeftTarget, int rearRightTarget, double power) {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Turn the motor back on, required if you use STOP_AND_RESET_ENCODER
        /*frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        */

        frontLeftMotor.setTargetPosition(frontLeftTarget);
        frontRightMotor.setTargetPosition(frontRightTarget);
        backLeftMotor.setTargetPosition(rearLeftTarget);
        backRightMotor.setTargetPosition(rearRightTarget);


        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);

        //imu.resetYaw();


        imu = hardwareMap.get(IMU.class, "imu");
        imu.resetYaw();

        while(frontLeftMotor.isBusy() && frontRightMotor.isBusy() &&
                backLeftMotor.isBusy() && backRightMotor.isBusy() && opModeIsActive()) {
            //Loop body can be empty
            // Get the current heading (yaw angle)
            telemetry.addData("distance is", sensorDistance.getDistance(DistanceUnit.INCH));
            if(sensorDistance.getDistance(DistanceUnit.INCH)<=15){
                int remainingCounts = frontLeftTarget-frontLeftMotor.getCurrentPosition();
                avoidObstacles(remainingCounts);
                break;
            }



            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double currentHeading = orientation.getYaw(AngleUnit.DEGREES);

            // Calculate the deviation (error) from the target heading
            double targetHeading = 0;
            double headingError = targetHeading - currentHeading;

            // Apply proportional correction: larger error means larger correction
            double HEADING_GAIN = 0.01;
            headingError = headingError*HEADING_GAIN;


            // Adjust motor powers based on the correction
            double frontLeftPower = power + headingError;
            double frontRightPower = power - headingError;
            double rearLeftPower = power - headingError;
            double rearRightPower = power + headingError;

            // Clip motor powers to stay within the valid range (-0.3 to 0.3)
            frontLeftPower = Range.clip(frontLeftPower, -0.3, 0.3);
            frontRightPower = Range.clip(frontRightPower, -0.3, 0.3);
            rearLeftPower = Range.clip(rearLeftPower, -0.3, 0.3);
            rearRightPower = Range.clip(rearRightPower, -0.3, 0.3);

            // Set motor powers
            frontLeftMotor.setPower(frontLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backLeftMotor.setPower(rearLeftPower);
            backRightMotor.setPower(rearRightPower);

            // Add telemetry for monitoring
            telemetry.addData("Target Heading", targetHeading);
            telemetry.addData("Current Heading", currentHeading);
            telemetry.addData("Correction", headingError);
            telemetry.addData("Front Left Power", frontLeftPower);
            telemetry.addData("Front Right Power", frontRightPower);
            telemetry.addData("Rear Left Power", rearLeftPower);
            telemetry.addData("Rear Right Power", rearRightPower);

            telemetry.update();
        }

        // Stop all motors after the timeout
       stopPower();
    }

    public void avoidObstacles(int remainingCounts) {
        stopPower();
        rotate(90);
        moveForward(24);
        rotate(-90);
        moveForward(36);
        rotate(-90);
        moveForward(24);
        rotate(90);
        moveForward(remainingCounts);
    }

    public void stopPower() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}