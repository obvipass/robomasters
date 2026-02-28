package org.firstinspires.ftc.teamcode.FRLib.hardware;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.Logger;

public class LimelightW {
    private final Limelight3A limelight;
    private final Logger logger;
    private int index;
    private boolean switchRequested;
    public final float MIN_TARGET_AREA = 0.01f;
    public final float fovX = 54.5f;
    public final float fovY = 42;

    public LimelightW(HardwareMap hardwareMap, int initialPipelineIndex, Logger logger) {
        this.logger = logger;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
    }

    public Limelight3A getLimelight() {
        return limelight;
    }

    public void switchPipeline(int index) {
        this.index = index;
        this.switchRequested = true;
        limelight.pipelineSwitch(index);
    }

    public int getPipelineIndex() {
        return this.getResult().getPipelineIndex();
    }

    public boolean didPipelineSwitch() {
        if (!this.switchRequested) return true;
        else if (this.getPipelineIndex() == this.index) {
            this.switchRequested = false;
            logger.logData("Pipeline", this.index);
            return true;
        } else {
            logger.logData("Pipeline", "Waiting for " + this.index);
            return false;
        }
    }

    public LLResult getResult() {
        return limelight.getLatestResult();
    }

    public LLResult getResultAndLog() {
        LLResult result = this.getResult();
        this.logResult(result);
        return result;
    }

    public void logResult() {
        LLResult result = this.getResult();
        logResult(result);
    }

    public void logResult(LLResult result) {
        if (result != null && result.isValid() && result.getTa() >= MIN_TARGET_AREA) {
            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)
            double ta = result.getTa(); // How big the target looks (0%-100% of the image)

            logger.clear();
            logger.logData("Limelight", "Target(s) Found");
            logger.logData("Target X", tx);
            logger.logData("Target Y", ty);
            logger.logData("Target Area", ta);

        } else {
            logger.logData("Limelight", "No Targets");
        }
    }
}

