package Vishruth;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumDriveTrain {

    final double pulsesPerRevolution = 537.7;
    final double wheelDiameterInches = 4;
    final double countsPerInch = pulsesPerRevolution / (wheelDiameterInches * Math.PI);
    final double countsPerDegree  = 11.05;

    Telemetry telemetry;

    HardwareMap mecanumMap;
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor rearLeftDrive;
    DcMotor rearRightDrive;

    MotorPower frontLeftDriveMP;
    MotorPower frontRightDriveMP;
    MotorPower rearLeftDriveMP;
    MotorPower rearRightDriveMP;

    public MecanumDriveTrain(HardwareMap mecanumMap, Telemetry telemetry) {
        this.mecanumMap = mecanumMap;
        this.telemetry = telemetry;
        this.telemetry.addData("Constructor","Ready");
        this.telemetry.update();
    }

    public void initDriveTrain() {

        //fills in the motor variables with actual motors from hw map
        frontLeftDrive = mecanumMap.get(DcMotor.class, "front_left_motor");
        frontRightDrive = mecanumMap.get(DcMotor.class, "front_right_motor");
        rearLeftDrive = mecanumMap.get(DcMotor.class, "back_left_motor");
        rearRightDrive = mecanumMap.get(DcMotor.class, "back_right_motor");
        telemetry.addData("Motor Variables","Initialized");
        telemetry.update();

        //makes all the motors brake when they are not sent any power
        setAllMotorZeroPowerBehaviorsTo(ZeroPowerBehavior.BRAKE);

        //sets the correct directions for the motors
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        rearLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        rearRightDrive.setDirection(DcMotor.Direction.FORWARD);
        telemetry.addData("Motor Directions", "Ready");

        frontLeftDriveMP = new MotorPower(frontLeftDrive, 1, -1);
        frontRightDriveMP = new MotorPower(frontRightDrive, 1, -1);
        rearLeftDriveMP = new MotorPower(rearLeftDrive, 1, -1);
        rearRightDriveMP = new MotorPower(rearRightDrive, 1, -1);
        telemetry.addData("MotorPower","Ready");
        telemetry.update();


    }

    public void stopAllMotors(){
        frontLeftDriveMP.setMotorPower(0);
        frontRightDriveMP.setMotorPower(0);
        rearRightDriveMP.setMotorPower(0);
        rearLeftDriveMP.setMotorPower(0);
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

        frontLeftDriveMP.setMotorPower(axial + lateral + yaw);
        frontRightDriveMP.setMotorPower(axial - lateral - yaw);
        rearLeftDriveMP.setMotorPower(axial - lateral + yaw);
        rearRightDriveMP.setMotorPower(axial + lateral - yaw);

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

        frontRightDriveMP.setMotorPower(Math.abs(speed));
        frontLeftDriveMP.setMotorPower(Math.abs(speed));
        rearRightDriveMP.setMotorPower(Math.abs(speed));
        rearLeftDriveMP.setMotorPower(Math.abs(speed));
        telemetry.addData("Speed",speed);
        telemetry.update();

        setAllMotorRunModesTo(DcMotor.RunMode.RUN_TO_POSITION);
        while (frontRightDrive.isBusy() && frontLeftDrive.isBusy() && rearLeftDrive.isBusy() && rearRightDrive.isBusy()){
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

        frontRightDriveMP.setMotorPower(Math.abs(speed));
        frontLeftDriveMP.setMotorPower(Math.abs(speed));
        rearRightDriveMP.setMotorPower(Math.abs(speed));
        rearLeftDriveMP.setMotorPower(Math.abs(speed));
        telemetry.addData("Speed",speed);
        telemetry.update();

        setAllMotorRunModesTo(DcMotor.RunMode.RUN_TO_POSITION);
        while (frontRightDrive.isBusy() && frontLeftDrive.isBusy() && rearLeftDrive.isBusy() && rearRightDrive.isBusy()){
            telemetry.addData("Moving","");
            telemetry.update();
        }

        stopAllMotors();
    }
}
