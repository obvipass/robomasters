package Vishruth.Library.Library.SubAssemblies;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import Vishruth.Library.Library.Hardware.Imu;
import Vishruth.Library.Library.Hardware.Motor;

public class MecanumDriveTrain {

    final double pulsesPerRevolution = 537.7;
    final double wheelDiameterInches = 4;
    final double countsPerInch = pulsesPerRevolution / (wheelDiameterInches * Math.PI);
    final double countsPerDegree = 11.06;
    double GSPK = 0.1;
    final double overshootPerInch = 1.036458333333333333;

    double KP = 0.1;
    double KI = 0.1;
    double KD = 0.1;

    Telemetry telemetry;
    LinearOpMode opMode;

    HardwareMap mecanumMap;

    public Imu imu = new Imu();

    public Motor frontLeftDrive;
    public Motor frontRightDrive;
    public Motor rearLeftDrive;
    public Motor rearRightDrive;

    //initializations
    public MecanumDriveTrain(@NonNull LinearOpMode opMode) {
        this.mecanumMap = opMode.hardwareMap;
        this.opMode = opMode;

        this.telemetry = opMode.telemetry;
        this.telemetry.addData("Constructor", "Ready");
        this.telemetry.update();

        initDriveTrain();
    }

    public void initDriveTrain() {

        //makes all the motors brake when they are not sent any power
        frontLeftDrive = new Motor(mecanumMap.get(DcMotor.class, "front_left_motor"), 1, -1);
        frontRightDrive = new Motor(mecanumMap.get(DcMotor.class, "front_right_motor"), 1, -1);
        rearLeftDrive = new Motor(mecanumMap.get(DcMotor.class, "back_left_motor"), 1, -1);
        rearRightDrive = new Motor(mecanumMap.get(DcMotor.class, "back_right_motor"), 1, -1);
        //sets the correct directions for the motors
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        rearLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        rearRightDrive.setDirection(DcMotor.Direction.FORWARD);
        telemetry.addData("Motor Directions", "Ready");

        setAllMotorZeroPowerBehaviorsTo(ZeroPowerBehavior.BRAKE);
        telemetry.addData("ZeroPowerBehavior", "Ready");
        telemetry.update();
        imu.initIMU(mecanumMap);

        telemetry.addData("MotorPower", "Ready");
        telemetry.update();


    }

    //getters
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        // Determine the heading current error
        double headingError = desiredHeading - imu.getHeading(AngleUnit.DEGREES);

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    public boolean allMotorsAreBusy() {
        return frontRightDrive.isBusy() && frontLeftDrive.isBusy() && rearLeftDrive.isBusy() && rearRightDrive.isBusy();
    }

    //all motor methods
    public void stopAllMotors() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        rearRightDrive.setPower(0);
        rearLeftDrive.setPower(0);
        telemetry.addData("Motors", "Stopped");
        telemetry.update();
    }

    public void setAllMotorRunModesTo(DcMotor.RunMode r) {
        frontLeftDrive.setMode(r);
        frontRightDrive.setMode(r);
        rearLeftDrive.setMode(r);
        rearRightDrive.setMode(r);
        telemetry.addData("Motor RunModes set to", r);
        telemetry.update();
    }

    public void setAllMotorZeroPowerBehaviorsTo(DcMotor.ZeroPowerBehavior z) {
        frontLeftDrive.setZeroPowerBehavior(z);
        frontRightDrive.setZeroPowerBehavior(z);
        rearLeftDrive.setZeroPowerBehavior(z);
        rearRightDrive.setZeroPowerBehavior(z);
        telemetry.addData("Motors ZeroPowerBehaviorSetTo", z);
        telemetry.update();
    }

    public void setAllMotorPowersTo(double power) {
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        rearRightDrive.setPower(power);
        rearLeftDrive.setPower(power);
        telemetry.addData("Power", power);
        telemetry.update();
    }

    public void brake(long holdTimeMs) {

        // ROBOT SHOULD ALREADY BE IN BRAKE ZERO-POWER MODE
        setAllMotorZeroPowerBehaviorsTo(DcMotor.ZeroPowerBehavior.BRAKE);

        // Read target hold positions (encoder ticks)
        int flTarget = frontLeftDrive.getCurrentPosition();
        int frTarget = frontRightDrive.getCurrentPosition();
        int rlTarget = rearLeftDrive.getCurrentPosition();
        int rrTarget = rearRightDrive.getCurrentPosition();

        double KP = 0.01;       // proportional gain for holding
        double maxHoldPower = 0.3;   // limit to protect motors

        ElapsedTime timer = new ElapsedTime();

        // Run lightweight PID loop
        while (opMode.opModeIsActive() && timer.milliseconds() < holdTimeMs) {

            // Compute position errors
            double flError = flTarget - frontLeftDrive.getCurrentPosition();
            double frError = frTarget - frontRightDrive.getCurrentPosition();
            double rlError = rlTarget - rearLeftDrive.getCurrentPosition();
            double rrError = rrTarget - rearRightDrive.getCurrentPosition();

            // Compute hold power based on P only (I and D not needed)
            double flPower = Range.clip(flError * KP, -maxHoldPower, maxHoldPower);
            double frPower = Range.clip(frError * KP, -maxHoldPower, maxHoldPower);
            double rlPower = Range.clip(rlError * KP, -maxHoldPower, maxHoldPower);
            double rrPower = Range.clip(rrError * KP, -maxHoldPower, maxHoldPower);

            // Apply holding power
            frontLeftDrive.setPower(flPower);
            frontRightDrive.setPower(frPower);
            rearLeftDrive.setPower(rlPower);
            rearRightDrive.setPower(rrPower);

            telemetry.addData("Braking", holdTimeMs);
            telemetry.update();

            opMode.idle();
        }

        // Stop applying hold power after time expires
        setAllMotorPowersTo(0);
    }
    //movement

    public void moveFieldRelative(double axial, double lateral, double yaw) {
        // convert axial and lateral into angle or direction of vector and magnitude or size of vector

        double magnitudeOfVector = Math.hypot(axial, lateral);
        double angle = Math.atan2(axial, lateral);

        // account for the direction of robot, add if right is positive
        angle = AngleUnit.normalizeRadians(angle - imu.getHeading(AngleUnit.RADIANS));

        //braking down angle and magnitude back into axial and lateral values
        double axialNew = magnitudeOfVector * Math.sin(angle);
        double lateralNew = magnitudeOfVector * Math.cos(angle);

        moveWithALY(axialNew, lateralNew, yaw);


    }

    public void moveWithALY(double axial, double lateral, double yaw) {

        /* double frontLeftPower  = axial + lateral + yaw
        double frontRightPower = axial - lateral - yaw
        double backLeftPower   = axial - lateral + yaw
        double backRightPower  = axial + lateral - yaw */

        setAllMotorZeroPowerBehaviorsTo(ZeroPowerBehavior.BRAKE);
        telemetry.addData("Axial", axial);
        telemetry.addLine();
        telemetry.addData("Lateral", lateral);
        telemetry.addLine();
        telemetry.addData("Yaw", yaw);
        telemetry.update();

        frontLeftDrive.setPower(axial + lateral + yaw);
        frontRightDrive.setPower(axial - lateral - yaw);
        rearLeftDrive.setPower(axial - lateral + yaw);
        rearRightDrive.setPower(axial + lateral - yaw);

    }

    public void moveInches(double speed, double leftInches, double rightInches) {

        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newRearLeftTarget;
        int newRearRightTarget;

        setAllMotorZeroPowerBehaviorsTo(ZeroPowerBehavior.BRAKE);

        newFrontLeftTarget = frontLeftDrive.getCurrentPosition() + (int) (leftInches * countsPerInch);
        newFrontRightTarget = frontRightDrive.getCurrentPosition() + (int) (rightInches * countsPerInch);
        newRearLeftTarget = rearLeftDrive.getCurrentPosition() + (int) (leftInches * countsPerInch);
        newRearRightTarget = rearRightDrive.getCurrentPosition() + (int) (rightInches * countsPerInch);
        telemetry.addData("targetPositions", "%i,%i,%i,%i", newFrontLeftTarget, newFrontLeftTarget, newRearRightTarget, newRearLeftTarget);
        telemetry.update();

        frontRightDrive.setTargetPosition(newFrontRightTarget);
        frontLeftDrive.setTargetPosition(newFrontLeftTarget);
        rearRightDrive.setTargetPosition(newRearRightTarget);
        rearLeftDrive.setTargetPosition(newRearLeftTarget);
        telemetry.addData("target positions", "inputted");
        telemetry.update();

        frontRightDrive.setPower(Math.abs(speed));
        frontLeftDrive.setPower(Math.abs(speed));
        rearRightDrive.setPower(Math.abs(speed));
        rearLeftDrive.setPower(Math.abs(speed));
        telemetry.addData("Speed", speed);
        telemetry.update();

        setAllMotorRunModesTo(DcMotor.RunMode.RUN_TO_POSITION);
        while (allMotorsAreBusy()) {
            telemetry.addData("Moving", "");
            telemetry.update();
        }

        stopAllMotors();
    }


    public void driveIMU(double GSPK, double distanceInches, double power) {

        int moveCounts = (int) (distanceInches * countsPerInch);
        imu.resetYaw();
        int newFrontLeftTarget = frontLeftDrive.getCurrentPosition() + moveCounts;
        int newFrontRightTarget = frontRightDrive.getCurrentPosition() + moveCounts;
        int newRearLeftTarget = frontLeftDrive.getCurrentPosition() + moveCounts;
        int newRearRightTarget = frontRightDrive.getCurrentPosition() + moveCounts;

        // Set target positions
        frontLeftDrive.setTargetPosition(newFrontLeftTarget);
        rearLeftDrive.setTargetPosition(newRearLeftTarget);
        frontRightDrive.setTargetPosition(newFrontRightTarget);
        rearRightDrive.setTargetPosition(newRearRightTarget);

        // Set to RUN_TO_POSITION mode
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Record the starting heading


        // Start motion
        frontLeftDrive.setPower(power);
        rearLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        rearRightDrive.setPower(power);

        // Proportional gain (tune this!)

        while (allMotorsAreBusy()) {

            double currentAngle = imu.getHeading(AngleUnit.DEGREES);
            double error = 0 - currentAngle;

            // Keep error in [-180, 180]
            if (error > 180) error -= 360;
            if (error < -180) error += 360;

            double correction = GSPK * error;

            double leftPower = power + correction;
            double rightPower = power - correction;

            // Apply corrected powers
            frontLeftDrive.setPower(leftPower);
            rearLeftDrive.setPower(leftPower);
            frontRightDrive.setPower(rightPower);
            rearRightDrive.setPower(rightPower);
        }

    }

    public void turnToAngle(double maxTurnSpeed, double heading, double proportional) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, proportional);

        // keep looping while we are still active, and not on heading
        double turnSpeed = getSteeringCorrection(heading, GSPK);
        moveRobot(0, turnSpeed);


        while (allMotorsAreBusy()) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, GSPK);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    public void driveStraight(double driveSpeed, double distance, double heading) {

        // Ensure that the OpMode is still active


        // Determine new target position, and pass to motor controller
        int moveCounts = (int) (distance * countsPerInch);
        int frontLeftDriveTarget = frontLeftDrive.getCurrentPosition() + moveCounts;
        int rearLeftDriveTarget = rearLeftDrive.getCurrentPosition() + moveCounts;
        int frontRightDriveTarget = frontRightDrive.getCurrentPosition() + moveCounts;
        int rearRightDriveTarget = rearRightDrive.getCurrentPosition() + moveCounts;

        // Set Target FIRST, then turn on RUN_TO_POSITION
        frontLeftDrive.setTargetPosition(frontLeftDriveTarget);
        rearLeftDrive.setTargetPosition(rearLeftDriveTarget);
        frontRightDrive.setTargetPosition(frontRightDriveTarget);
        rearRightDrive.setTargetPosition(rearRightDriveTarget);

        setAllMotorRunModesTo(DcMotor.RunMode.RUN_TO_POSITION);

        // Set the required driving speed  (must be positive for RUN_TO_POSITION)
        // Start driving straight, and then enter the control loop
        moveRobot(driveSpeed, 0);
        double turnSpeed;
        double distanceRemaining;
        // keep looping while we are still active, and BOTH motors are running.
        while (allMotorsAreBusy() && opMode.opModeIsActive()) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, GSPK);


            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0)
                turnSpeed *= -1.0;

            // Apply the turning correction to the current driving speed.
            moveRobot(driveSpeed, turnSpeed);

            // Display drive status for the driver.


            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            setAllMotorRunModesTo(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void moveRobot(double drive, double turn) {

        double leftSpeed = drive - turn;
        double rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0) {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        frontLeftDrive.setPower(leftSpeed);
        rearLeftDrive.setPower(leftSpeed);
        frontRightDrive.setPower(rightSpeed);
        rearRightDrive.setPower(rightSpeed);
    }

    public void turnToAnglePID(double targetAngle, double maxPower) {
        double Kp = 0.03;
        double Ki = 0.0;
        double Kd = 0.002;

        double error;
        double integral = 0;
        double previousError = 0;
        double derivative;

        long previousTime = System.currentTimeMillis();

        while (opMode.opModeIsActive()) {
            double currentAngle = imu.getHeading(AngleUnit.DEGREES); // from IMU
            error = targetAngle - currentAngle;

            // Keep error in [-180, 180]
            if (error > 180) error -= 360;
            if (error < -180) error += 360;

            // Time since last loop
            long currentTime = System.currentTimeMillis();
            double deltaTime = (currentTime - previousTime) / 1000.0;

            // PID calculations
            integral += error * deltaTime;
            derivative = (error - previousError) / deltaTime;

            double output = (Kp * error) + (Ki * integral) + (Kd * derivative);

            // Apply power to motors
            frontLeftDrive.setPower(output);
            rearLeftDrive.setPower(output);
            frontRightDrive.setPower(-output);
            rearRightDrive.setPower(-output);

            // Break when close enough
            if (Math.abs(error) <= 1.0) break;

            previousError = error;
            previousTime = currentTime;

            telemetry.addData("Target", targetAngle);
            telemetry.addData("Heading", currentAngle);
            telemetry.addData("Error", error);
            telemetry.addData("Output", output);
            telemetry.update();
        }

        stopAllMotors();
    }

    public void induceError(Motor motor, double power, double timeInMilliseconds) {
        ElapsedTime time = new ElapsedTime();
        while (time.milliseconds() < timeInMilliseconds) {
            motor.setPower(power);
        }
        setAllMotorPowersTo(0);


    }

    public void driveStraightWithDistanceControl(double targetDistance, double maxPower, double heading) {
        targetDistance /= overshootPerInch;
        setAllMotorRunModesTo(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setAllMotorZeroPowerBehaviorsTo(ZeroPowerBehavior.BRAKE);
        imu.resetYaw();
        setAllMotorRunModesTo(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Motors", "Ready");
        telemetry.update();

        double newFrontLeftTarget = frontLeftDrive.getCurrentPosition() + (int) (targetDistance * countsPerInch);
        double newFrontRightTarget = frontRightDrive.getCurrentPosition() + (int) (targetDistance * countsPerInch);
        double newRearLeftTarget = rearLeftDrive.getCurrentPosition() + (int) (targetDistance * countsPerInch);
        double newRearRightTarget = rearRightDrive.getCurrentPosition() + (int) (targetDistance * countsPerInch);

        telemetry.addData("targetPositions", "%i,%i,%i,%i", newFrontLeftTarget, newFrontLeftTarget, newRearRightTarget, newRearLeftTarget);
        telemetry.update();

        frontRightDrive.setTargetPosition((int) newFrontRightTarget);
        frontLeftDrive.setTargetPosition((int) newFrontLeftTarget);
        rearRightDrive.setTargetPosition((int) newRearRightTarget);
        rearLeftDrive.setTargetPosition((int) newRearLeftTarget);

        telemetry.addData("TargetPos", "Inputted");
        telemetry.update();

        setAllMotorRunModesTo(DcMotor.RunMode.RUN_TO_POSITION);

        double minPower = 0.2;
        double startPos = frontLeftDrive.getCurrentPosition();
        double sumOfErrors = 0;
        double previousError = 0;

        ElapsedTime time = new ElapsedTime();

        while (opMode.opModeIsActive() && allMotorsAreBusy()) {

            // Calculate distance traveled
            double currentPos = frontLeftDrive.getCurrentPosition();
            double distanceTraveled = (currentPos - startPos) / countsPerInch;

            // Signed error
            double error = targetDistance - distanceTraveled;

            if (Math.abs(error) < 5) {
                sumOfErrors += error;
            } else {
                sumOfErrors = 0;
            }

            // PID output
            double drivePower = calculatePIDPower(error, sumOfErrors, previousError);

            // Clip power
            drivePower = Range.clip(drivePower, minPower, maxPower);

            telemetry.addData("DrivePower", drivePower);
            telemetry.update();

            // Heading correction
            double turnCorrection = getSteeringCorrection(heading, 0.05);

            telemetry.addData("TurnPower", turnCorrection);
            telemetry.update();

            // Move robot
            moveRobot(drivePower, turnCorrection);

            telemetry.addData("Target", targetDistance);
            telemetry.addData("Traveled", distanceTraveled);
            telemetry.addData("Error", error);
            telemetry.addData("Power", drivePower);
            telemetry.update();

            previousError = error;
        }
        brake(1000);
        setAllMotorPowersTo(0);
        setAllMotorRunModesTo(DcMotor.RunMode.RUN_USING_ENCODER);
        opMode.sleep(1000);
        opMode.idle();

        moveRobot(0, 0);
    }

    public void driveStraightWithDistanceControlRunToPos(double targetDistance, double maxPower, double heading) {

    }

    public double calculatePIDPower(double error, double sumOfAllPastErrors, double previousError) {
        double proportionalCorrection = error * KP;
        double integralCorrection = sumOfAllPastErrors * KI;
        double derivativeCorrection = (previousError - error) * KD;

        return proportionalCorrection + Range.clip(integralCorrection, -0.5, 0.5) + derivativeCorrection;


    }

    // commented out code

    //    public void moveInchesWithCOC(double speed, double leftInches, double rightInches) {
//
//        int newFrontLeftTarget;
//        int newFrontRightTarget;
//        int newRearLeftTarget;
//        int newRearRightTarget;
//
//        double constantOvershoot = 1.65;
//
//        setAllMotorZeroPowerBehaviorsTo(ZeroPowerBehavior.BRAKE);
//
//        newFrontLeftTarget = frontLeftDrive.getCurrentPosition() + (int) ((leftInches-constantOvershoot) * countsPerInch);
//        newFrontRightTarget = frontRightDrive.getCurrentPosition() + (int) ((rightInches-constantOvershoot) * countsPerInch);
//        newRearLeftTarget = rearLeftDrive.getCurrentPosition() + (int) ((leftInches - constantOvershoot) * countsPerInch);
//        newRearRightTarget = rearRightDrive.getCurrentPosition() + (int) ((rightInches-constantOvershoot) * countsPerInch);
//        telemetry.addData("targetPositions","%i,%i,%i,%i",newFrontLeftTarget,newFrontLeftTarget,newRearRightTarget,newRearLeftTarget);
//        telemetry.update();
//
//        frontRightDrive.setTargetPosition(newFrontRightTarget);
//        frontLeftDrive.setTargetPosition(newFrontLeftTarget);
//        rearRightDrive.setTargetPosition(newRearRightTarget);
//        rearLeftDrive.setTargetPosition(newRearLeftTarget);
//        telemetry.addData("target positions","inputted");
//        telemetry.update();
//
//        frontRightDrive.setPower(Math.abs(speed));
//        frontLeftDrive.setPower(Math.abs(speed));
//        rearRightDrive.setPower(Math.abs(speed));
//        rearLeftDrive.setPower(Math.abs(speed));
//        telemetry.addData("Speed",speed);
//        telemetry.update();
//
//        setAllMotorRunModesTo(DcMotor.RunMode.RUN_TO_POSITION);
//        while (allMotorsAreBusy()){
//            telemetry.addData("Moving","");
//            telemetry.update();
//        }
//
//        stopAllMotors();
//    }

//    public void moveInchesWithCPOC(double speed, double leftInches, double rightInches) {
//
//        int newFrontLeftTarget;
//        int newFrontRightTarget;
//        int newRearLeftTarget;
//        int newRearRightTarget;
//
//        double constantOvershoot = 0.5;
//        double proportionalOvershootPerInch = 0.1;
//        double correction = 1 - (1/(1 + proportionalOvershootPerInch));
//
//        setAllMotorZeroPowerBehaviorsTo(ZeroPowerBehavior.BRAKE);
//
//        newFrontLeftTarget = frontLeftDrive.getCurrentPosition() + (int) ((leftInches*correction-constantOvershoot) * countsPerInch);
//        newFrontRightTarget = frontRightDrive.getCurrentPosition() + (int) ((rightInches*correction-constantOvershoot) * countsPerInch);
//        newRearLeftTarget = rearLeftDrive.getCurrentPosition() + (int) ((leftInches*correction-constantOvershoot) * countsPerInch);
//        newRearRightTarget = rearRightDrive.getCurrentPosition() + (int) ((rightInches-constantOvershoot) * countsPerInch);
//        telemetry.addData("targetPositions","%i,%i,%i,%i",newFrontLeftTarget,newFrontLeftTarget,newRearRightTarget,newRearLeftTarget);
//        telemetry.update();
//
//        frontRightDrive.setTargetPosition(newFrontRightTarget);
//        frontLeftDrive.setTargetPosition(newFrontLeftTarget);
//        rearRightDrive.setTargetPosition(newRearRightTarget);
//        rearLeftDrive.setTargetPosition(newRearLeftTarget);
//        telemetry.addData("target positions","inputted");
//        telemetry.update();
//
//        frontRightDrive.setPower(Math.abs(speed));
//        frontLeftDrive.setPower(Math.abs(speed));
//        rearRightDrive.setPower(Math.abs(speed));
//        rearLeftDrive.setPower(Math.abs(speed));
//        telemetry.addData("Speed",speed);
//        telemetry.update();
//
//        setAllMotorRunModesTo(DcMotor.RunMode.RUN_TO_POSITION);
//        while (allMotorsAreBusy()){
//            telemetry.addData("Moving","");
//            telemetry.update();
//        }
//
//        stopAllMotors();
//    }

     /*public void activelyResistChange(double milliseconds) {

        // Hold current position
        int frontLeftTarget = frontLeftDrive.getCurrentPosition();
        int frontRightTarget = frontRightDrive.getCurrentPosition();
        int rearRightTarget = rearRightDrive.getCurrentPosition();
        int rearLeftTarget = rearLeftDrive.getCurrentPosition();

        // Must be in RUN_TO_POSITION
        frontLeftDrive.setTargetPosition(frontLeftTarget);
        frontRightDrive.setTargetPosition(frontRightTarget);
        rearLeftDrive.setTargetPosition(rearLeftTarget);
        rearRightDrive.setTargetPosition(rearRightTarget);
        setAllMotorRunModesTo(DcMotor.RunMode.RUN_TO_POSITION);


        // Power needed to hold position
        setAllMotorPowersTo(0.3);  // adjust if needed

        ElapsedTime timer = new ElapsedTime();

        while (timer.milliseconds() < milliseconds) {

            int frontLeftDriveCurrentPosition = frontLeftDrive.getCurrentPosition();
            int frontRightDriveCurrentPosition = frontRightDrive.getCurrentPosition();
            int rearRightDriveCurrentPosition = rearRightDrive.getCurrentPosition();
            int rearLeftDriveCurrentPosition = rearLeftDrive.getCurrentPosition();

            double frontLeftError = frontLeftTarget - frontLeftDriveCurrentPosition;
            double frontRightError = frontRightTarget - frontRightDriveCurrentPosition;
            double rearLeftError = rearLeftTarget - rearLeftDriveCurrentPosition;
            double rearRightError = rearRightTarget - rearRightDriveCurrentPosition;

            // If pushed, command motor back to position
            if (Math.abs(frontLeftError) > 5 && Math.abs(rearLeftError)>5 && Math.abs(frontRightError) > 5 && Math.abs(rearRightError) > 5) {   // deadband of 5 ticks
                frontLeftDrive.setTargetPosition(frontLeftTarget);
                frontRightDrive.setTargetPosition(frontRightTarget);
                rearLeftDrive.setTargetPosition(rearLeftTarget);
                rearRightDrive.setTargetPosition(rearRightTarget);
                opMode.idle();
            }

        }

        // Stop holding
        setAllMotorPowersTo(0);
        setAllMotorRunModesTo(DcMotor.RunMode.RUN_USING_ENCODER);
    }*/

     /*public void driveStraightWithDistanceControl(double targetDistance, double maxPower, double heading) {
        // Reset encoders
        setAllMotorRunModesTo(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        imu.resetYaw();

        telemetry.addData("Yaw","Reset");
        telemetry.update();

        setAllMotorRunModesTo(DcMotor.RunMode.RUN_USING_ENCODER);


        double minPower = 0.2;

        double startPosition = frontLeftDrive.getCurrentPosition();
        double distanceTraveled;
        double distanceRemaining;
        double sumOFErrors = 0;
        double previousError = 0;

        while (opMode.opModeIsActive()) {  // stop when close

            // Update distance traveled
            double currentPosition = frontLeftDrive.getCurrentPosition();
            distanceTraveled = (currentPosition - startPosition) / countsPerInch;
            distanceRemaining = targetDistance - distanceTraveled;
            sumOFErrors = sumOFErrors + distanceRemaining;


            double drivePower = Math.min(calculatePIDPower(distanceRemaining,sumOFErrors,previousError),maxPower);




            // Use IMU to correct heading drift
            double turnCorrection = getSteeringCorrection(heading, 0.05);

            // Move the robot
            moveRobot(drivePower, turnCorrection);

            // Telemetry feedback
            telemetry.addData("Target (in)", targetDistance);
            telemetry.addData("Traveled (in)", distanceTraveled);
            telemetry.addData("Remaining (in)", distanceRemaining);
            telemetry.addData("Power", drivePower);
            telemetry.update();

            if(Math.abs(distanceRemaining)<0.25){
                break;
            }

            previousError = distanceRemaining;
        }

        // Stop all motors
        moveRobot(0, 0);
    }*/

    /*public void driveWithIMUStraight(double GSPK, double Seconds, double power,double TargetAngle){

        imu.resetYaw();

        // Set target positions


        // Set to RUN_TO_POSITION mode
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (frontLeftDrive.isBusy && frontRightDrive.isBusy && rearLeftDrive.isBusy && rearRightDrive.isBusy) {

            double currentAngle = imu.getHeading(AngleUnit.DEGREES);
            double error = TargetAngle - currentAngle;

            // Keep error in [-180, 180]
            if (error > 180) error -= 360;
            if (error < -180) error += 360;

            double correction = GSPK * error;

            double leftPower = power + correction;
            double rightPower = power - correction;

            // Apply corrected powers
            frontLeftDrive.setMotorPower(leftPower);
            rearLeftDrive.setMotorPower(leftPower);
            frontRightDrive.setMotorPower(rightPower);
            rearRightDrive.setMotorPower(rightPower);
        }

    }*/

    /*
    public void moveDegrees(int degrees,double speed){
        double newFrontLeftTarget;
        double newFrontRightTarget;
        double newRearLeftTarget;
        double newRearRightTarget;

        setAllMotorZeroPowerBehaviorsTo(ZeroPowerBehavior.BRAKE);

        newFrontLeftTarget = frontLeftDrive.getCurrentPosition() + (degrees * countsPerDegree);
        newFrontRightTarget = frontLeftDrive.getCurrentPosition() + (-degrees * countsPerDegree);
        newRearLeftTarget = frontLeftDrive.getCurrentPosition() + (degrees * countsPerDegree);
        newRearRightTarget = frontLeftDrive.getCurrentPosition() + (-degrees * countsPerDegree);
        telemetry.addData("targetPositions","%i,%i,%i,%i",newFrontLeftTarget,newFrontLeftTarget,newRearRightTarget,newRearLeftTarget);
        telemetry.update();

        frontRightDrive.setTargetPosition((int)newFrontRightTarget);
        frontLeftDrive.setTargetPosition((int)newFrontLeftTarget);
        rearRightDrive.setTargetPosition((int)newRearRightTarget);
        rearLeftDrive.setTargetPosition((int)newRearLeftTarget);
        telemetry.addData("target positions","inputted");
        telemetry.update();

        frontRightDrive.setPower(Math.abs(speed));
        frontLeftDrive.setPower(Math.abs(speed));
        rearRightDrive.setPower(Math.abs(speed));
        rearLeftDrive.setPower(Math.abs(speed));
        telemetry.addData("Speed",speed);
        telemetry.update();

        setAllMotorRunModesTo(DcMotor.RunMode.RUN_TO_POSITION);
        while (allMotorsAreBusy()){
            telemetry.addData("Moving","");
            telemetry.update();
        }

        stopAllMotors();
    } */

}
