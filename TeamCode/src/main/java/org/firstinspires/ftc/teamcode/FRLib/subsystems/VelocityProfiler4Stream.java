package org.firstinspires.ftc.teamcode.FRLib.subsystems;

import com.qualcomm.robotcore.util.Range;

public class VelocityProfiler4Stream {

    //double value = 0;



    double smoothingFactor;

    double currentValue1 = 0;
    double currentValue2 = 0;
    double currentValue3 = 0;
    double currentValue4 = 0;


    public VelocityProfiler4Stream(double smoothingFactor){
    this.smoothingFactor = Range.clip(smoothingFactor,0.0,1.0);
    }


    public double velocityProfileInput1(double inputTarget){
        currentValue1 = currentValue1 + (inputTarget - currentValue1) * smoothingFactor;
        return currentValue1;
    }
    public double velocityProfileInput2(double inputTarget){
        currentValue2 = currentValue2 + (inputTarget - currentValue2) * smoothingFactor;
        return currentValue2;
    }
    public double velocityProfileInput3(double inputTarget){
        currentValue3 = currentValue3 + (inputTarget - currentValue3) * smoothingFactor;
        return currentValue3;
    }

    public double velocityProfileInput4(double inputTarget){
        currentValue4 = currentValue4 + (inputTarget - currentValue4) * smoothingFactor;
        return currentValue4;
    }
// double power = velocityProfileIncrement(gamepad1.left_stick_y)

}
