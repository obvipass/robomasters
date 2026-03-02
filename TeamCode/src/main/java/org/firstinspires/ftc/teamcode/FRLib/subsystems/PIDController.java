package org.firstinspires.ftc.teamcode.FRLib.subsystems;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utils.Logger;

public class PIDController {
    private double minOutput = -1.0;
    private double maxOutput =  1.0;
    private final double maxIntegral = 1;
    private final double minIntegral = -1;
    private double kP, kI, kD;
    private double prevError = 0.0;
    private double integral = 0.0;

    private boolean firstUpdate = true;
    private long lastTimeNanos = 0;

    private final Logger logger;

    public PIDController(double kP, double kI, double kD, Logger logger) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        this.logger = logger;
    }

    public double getOutput(double error) {
        long now = System.nanoTime();

        logger.logData(Logger.LoggerMode.CRITICAL, "Error", error);

        double dt;
        if (lastTimeNanos == 0) {
            dt = 0.0;
        } else {
            dt = (now - lastTimeNanos) / 1e9; // divide by 1,000,000,000 to convert from nanoseconds to seconds
        }
        lastTimeNanos = now;

        double derivative;

        if (dt <= 0 || firstUpdate) {
            derivative = 0.0;
            firstUpdate = false;
        } else {
            derivative = (error - prevError) / dt;
        }

        integral += error * dt;
        integral = Range.clip(integral, minIntegral, maxIntegral);

        double output = (kP * error) + (kI * integral) + (kD * derivative);
        output = Range.clip(output, minOutput, maxOutput);

        prevError = error;


        return output;
    }

    public double getOutput(double target, double current) {
        logger.logData(Logger.LoggerMode.DETAILED, "PID Target", target);
        logger.logData(Logger.LoggerMode.DETAILED, "PID Current", current);

        return this.getOutput(target - current);
    }

    public void reset() {
        prevError = 0.0;
        integral = 0.0;
        firstUpdate = true;
        lastTimeNanos = 0;
    }

    public void setOutputLimits(double min, double max) {
        this.minOutput = min;
        this.maxOutput = max;
    }

    public void setKp(double kP) { this.kP = kP; }
    public void setKi(double kI) { this.kI = kI; }
    public void setKd(double kD) { this.kD = kD; }

    public double getKp() { return kP; }
    public double getKi() { return kI; }
    public double getKd() { return kD; }
}
