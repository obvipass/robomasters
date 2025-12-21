package org.firstinspires.ftc.teamcode.members.prathika;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Prathika - Avoidance")
public class AvoidAnnoyingObject extends LinearOpMode {

    // Motors
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;


    DistanceSensor distanceSensor;

    // Threshold in CHEESINESS

    static final double COUNTS_PER_DEGREE = 11.06;
    static final double THRESHOLD = 10.0;
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // 5203 Series Yellow -Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() {



        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_motor");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_motor");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_motor");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_motor");


        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance_sensor");


        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);


        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {

            double distance = distanceSensor.getDistance(DistanceUnit.INCH);

            telemetry.addData("Distance (IN)", "%.1f", distance);
            telemetry.update();

            // If object detected
            if (distance < THRESHOLD) {

                stopMotors();
                avoidObject();

                break;

            } else {
                // Otherwise, drive straight
                driveStraightInches(0.2, 10);
            }

            idle();
        }

        stopMotors();
    }
    public void driveStraightInches(double power, double inches) {

        int moveCounts = (int) (inches * COUNTS_PER_INCH);

        // Reset encoders
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set target positions
        frontLeftDrive.setTargetPosition(moveCounts);
        frontRightDrive.setTargetPosition(moveCounts);
        backLeftDrive.setTargetPosition(moveCounts);
        backRightDrive.setTargetPosition(moveCounts);

        // Run to position
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Apply power
        frontLeftDrive.setPower(Math.abs(power));
        frontRightDrive.setPower(Math.abs(power));
        backLeftDrive.setPower(Math.abs(power));
        backRightDrive.setPower(Math.abs(power));

        // Wait until all motors reach target
        while (opModeIsActive()
                && frontLeftDrive.isBusy()
                && frontRightDrive.isBusy()
                && backLeftDrive.isBusy()
                && backRightDrive.isBusy()) {

            telemetry.addData("Target", moveCounts);
            telemetry.addData("FL", frontLeftDrive.getCurrentPosition());
            telemetry.update();

            idle();
        }

        stopMotors();

        // Back to normal encoder mode
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void encoderDrive(double speed, double degrees,
                             double timeoutS) {

        double newFrontLeftTarget;
        double newFrontRightTarget;

        double newBackLeftTarget;
        double newBackRightTarget;

        newFrontLeftTarget = frontLeftDrive.getCurrentPosition() + (degrees * COUNTS_PER_DEGREE);
        newFrontRightTarget = frontRightDrive.getCurrentPosition() + (-degrees * COUNTS_PER_DEGREE);
        newBackLeftTarget = backLeftDrive.getCurrentPosition() + (degrees * COUNTS_PER_DEGREE);
        newBackRightTarget = backRightDrive.getCurrentPosition() + (-degrees * COUNTS_PER_DEGREE);

        if (opModeIsActive()) {



            frontLeftDrive.setTargetPosition((int)newFrontLeftTarget);
            frontRightDrive.setTargetPosition((int)newFrontRightTarget);
            backLeftDrive.setTargetPosition((int)newBackLeftTarget);
            backRightDrive.setTargetPosition((int)newBackRightTarget);


            // Turn On RUN_TO_POSITION
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            frontLeftDrive.setPower(Math.abs(speed));
            frontRightDrive.setPower(Math.abs(speed));
            backLeftDrive.setPower(Math.abs(speed));
            backRightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frontLeftDrive.isBusy() && frontRightDrive.isBusy()
                            && backLeftDrive.isBusy() && backRightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " FrontLeft: %7d FrontRight:%7d  BackLeft: %7d BackRight:%7d",
                        newFrontLeftTarget,  newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                telemetry.addData("Currently at",  " FrontLeft: %7d FrontRight:%7d  BackLeft: %7d BackRight:%7d",
                        frontLeftDrive.getCurrentPosition(), frontRightDrive.getCurrentPosition(),
                        backLeftDrive.getCurrentPosition(), backRightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);


            // Turn off RUN_TO_POSITION
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
   /*public void turnDegrees(double power, double degrees) {

        int moveCounts = (int) (degrees * COUNTS_PER_DEGREE);

        // Reset encoders
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set target positions
        frontLeftDrive.setTargetPosition(moveCounts);
        backLeftDrive.setTargetPosition(moveCounts);
        frontRightDrive.setTargetPosition(-moveCounts);
        backRightDrive.setTargetPosition(-moveCounts);

        // Run to position
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start motion
        frontLeftDrive.setPower(Math.abs(power));
        frontRightDrive.setPower(Math.abs(power));
        backLeftDrive.setPower(Math.abs(power));
        backRightDrive.setPower(Math.abs(power));

        // Wait for motors to finish
        while (opModeIsActive()
                && frontLeftDrive.isBusy()
                && frontRightDrive.isBusy()
                && backLeftDrive.isBusy()
                && backRightDrive.isBusy()) {
            idle();
        }*/

        stopMotors();

        // Back to normal mode
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Stop all motors
    public void stopMotors() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }


    public void avoidObject() {

        frontLeftDrive.setPower(0.25);
        backLeftDrive.setPower(0.25);
        frontRightDrive.setPower(-0.25);
        backRightDrive.setPower(-0.25);
        sleep(600);



        driveStraightInches(0.1,24);

        driveStraightInches(0.1,24 );

        driveStraightInches(0.1,24);

        driveStraightInches(0.2,24);
        sleep(800);

        stopMotors();
    }
}
