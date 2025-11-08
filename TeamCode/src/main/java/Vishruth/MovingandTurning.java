package Vishruth;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import Vishruth.Libray.Hardware.MecanumDriveTrain;

public class MovingandTurning extends LinearOpMode {

    public enum States {
        MOVING_FORWARD,
        TURNING_RIGHT
    }

    States states = States.MOVING_FORWARD;

    MecanumDriveTrain mecanumDriveTrain;

    public void runOpMode(){
        mecanumDriveTrain = new MecanumDriveTrain(hardwareMap,telemetry);
        mecanumDriveTrain.initDriveTrain();

        waitForStart();
        int counter = 0;
        while (opModeIsActive()){
            switch (states){
                case MOVING_FORWARD:
                    mecanumDriveTrain.moveInches(0.2,60,60);
                    if(counter == 1){
                        requestOpModeStop();
                        break;
                    } else {
                      counter++;
                      states = States.TURNING_RIGHT;
                      break;
                    }
                case TURNING_RIGHT:
                    mecanumDriveTrain.moveDegrees(90,0.2);
                    states = States.MOVING_FORWARD;
                    break;
            }
        }


    }




}
