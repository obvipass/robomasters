package Vishruth;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import Vishruth.Library.Library.Hardware.Motor;

@TeleOp
public class VishruthCodeTest extends LinearOpMode {
    private ElapsedTime runTime = new ElapsedTime();
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor rearRightDrive;
    private DcMotor rearLeftDrive;

    private Motor frontLeftDriveMP;
    private Motor frontRightDriveMP;
    private Motor rearLeftDriveMP;
    private Motor rearRightDriveMP;


    private void move(double axial, double lateral, double yaw){
        /* double frontLeftPower  = axial + lateral + yaw
        double frontRightPower = axial - lateral - yaw
        double backLeftPower   = axial - lateral + yaw
        double backRightPower  = axial + lateral - yaw */

        frontLeftDriveMP.setPower(axial + lateral + yaw);
        frontRightDriveMP.setPower(axial - lateral - yaw);
        rearLeftDriveMP.setPower(axial - lateral + yaw);
        rearRightDriveMP.setPower(axial + lateral - yaw);

    }
    @Override
    public void runOpMode() {

        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        rearLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        rearRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        rearLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        rearRightDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status:","MotorsDirectionsReady");
        telemetry.update();

        frontLeftDriveMP = new Motor(frontLeftDrive, 0.2, -0.2);
        frontRightDriveMP = new Motor(frontRightDrive, 0.2, -0.2);
        rearLeftDriveMP = new Motor(rearLeftDrive, 0.2, -0.2);
        rearRightDriveMP = new Motor(rearRightDrive, 0.2, -0.2);

        telemetry.addData("Status:","MotorsReady");
        telemetry.update();

        waitForStart();
        runTime.reset();

        while(opModeIsActive()){

            move(1,0,0);
            sleep(3000);

            telemetry.addData("Moving","Forward");
            telemetry.update();

            move(-1,0,0);
            sleep(3000);

            telemetry.addData("Moving","Backward");
            telemetry.update();

            move(0,0,1);
            sleep(2000);

            telemetry.addData("Moving","Clockwise");
            telemetry.update();

            move(0,0,-1);
            sleep(2000);

            telemetry.addData("Moving","Anticlockwise");
            telemetry.update();

            move(0,1,0);
            sleep(3000);

            telemetry.addData("Moving","Right");
            telemetry.update();

            move(0,-1,0);
            sleep(3000);

            telemetry.addData("Moving","Left");
            telemetry.update();

        }

    }

}

