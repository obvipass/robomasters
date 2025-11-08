package Vishruth.Libray.Library.SubAssemblies;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import Vishruth.Libray.Library.Hardware.Imu;
import Vishruth.Libray.Library.Hardware.Motor;

public class MecanumDriveTrain {

    final double pulsesPerRevolution = 537.7;
    final double wheelDiameterInches = 4;
    final double countsPerInch = pulsesPerRevolution / (wheelDiameterInches * Math.PI);
    final double countsPerDegree  = 11.06;
    double GSPK = 0.1;
    Telemetry telemetry;
    OpMode opMode;

    HardwareMap mecanumMap;


    Imu imu;

    Motor frontLeftDrive;
    Motor frontRightDrive;
    Motor rearLeftDrive;
    Motor rearRightDrive;

    public MecanumDriveTrain(HardwareMap mecanumMap, OpMode opMode) {
        this.mecanumMap = mecanumMap;
        this.opMode = opMode;

        this.telemetry = opMode.telemetry;
        this.telemetry.addData("Constructor","Ready");
        this.telemetry.update();
    }





    public void initDriveTrain() {

        //makes all the motors brake when they are not sent any power
        setAllMotorZeroPowerBehaviorsTo(ZeroPowerBehavior.BRAKE);
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

        imu.initIMU(mecanumMap);

        telemetry.addData("MotorPower","Ready");
        telemetry.update();


    }

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        // Determine the heading current error
        double headingError = desiredHeading - imu.getHeading(AngleUnit.DEGREES);

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    public boolean allMotorsAreBusy(){
        return frontRightDrive.isBusy && frontLeftDrive.isBusy && rearLeftDrive.isBusy && rearRightDrive.isBusy;
    }

    public void stopAllMotors(){
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        rearRightDrive.setPower(0);
        rearLeftDrive.setPower(0);
        telemetry.addData("Motors","Stopped");
        telemetry.update();
    }

    public void setAllMotorRunModesTo(DcMotor.RunMode r){
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
        telemetry.addData("Motors ZeroPowerBehaviorSetTo", z );
        telemetry.update();
    }


    private void moveWithALY(double axial, double lateral, double yaw){

        /* double frontLeftPower  = axial + lateral + yaw
        double frontRightPower = axial - lateral - yaw
        double backLeftPower   = axial - lateral + yaw
        double backRightPower  = axial + lateral - yaw */

        setAllMotorZeroPowerBehaviorsTo(ZeroPowerBehavior.BRAKE);
        telemetry.addData("Axial",axial);
        telemetry.addLine();
        telemetry.addData("Lateral",lateral);
        telemetry.addLine();
        telemetry.addData("Yaw",yaw);
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
        telemetry.addData("targetPositions","%i,%i,%i,%i",newFrontLeftTarget,newFrontLeftTarget,newRearRightTarget,newRearLeftTarget);
        telemetry.update();

        frontRightDrive.setTargetPosition(newFrontRightTarget);
        frontLeftDrive.setTargetPosition(newFrontLeftTarget);
        rearRightDrive.setTargetPosition(newRearRightTarget);
        rearLeftDrive.setTargetPosition(newRearLeftTarget);
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
    }

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
    }

    public void driveWithIMUStraight(double GSPK, double distanceInches, double power){
        int moveCounts = (int) (distanceInches * countsPerInch);
        imu.resetYaw();
        int newFrontLeftTarget = frontLeftDrive.getCurrentPosition() + moveCounts;
        int newFrontRightTarget = frontRightDrive.getCurrentPosition() + moveCounts;
        int newRearLeftTarget = frontLeftDrive.getCurrentPosition() + moveCounts;
        int newRearRightTarget = frontRightDrive.getCurrentPosition() + moveCounts;

        // Set target positions
        frontLeftDrive.setTargetPosition(newFrontLeftTarget);
        rearLeftDrive.setTargetPosition(newRearLeftTarget);
        frontRightDrive.setTargetPosition(newFrontLeftTarget);
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

        while (frontLeftDrive.isBusy && frontRightDrive.isBusy && rearLeftDrive.isBusy && rearRightDrive.isBusy) {

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


    public void turnToAngle(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, GSPK);

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

    public void driveStraight(double driveSpeed,
                              double distance,
                              double heading) {

        // Ensure that the OpMode is still active


            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * countsPerInch);
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
            // keep looping while we are still active, and BOTH motors are running.
            while (allMotorsAreBusy()) {

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

        double leftSpeed  = drive - turn;
        double rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        frontLeftDrive.setPower(leftSpeed);
        rearLeftDrive.setPower(leftSpeed);
        frontRightDrive.setPower(rightSpeed);
        rearRightDrive.setPower(rightSpeed);
    }

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

}
