package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.Range;
import java.util.HashMap;
import java.util.Map;

public class VelocityProfilerDynamic {

    private final double smoothingFactor;
    private final Map<String, Double> inputs = new HashMap<>();

    public VelocityProfilerDynamic(double smoothingFactor) {
        this.smoothingFactor = Range.clip(smoothingFactor, 0.0, 1.0);
    }

    public void addInput(String inputName) {
        if (inputs.containsKey(inputName)) {
            throw new IllegalArgumentException(
                    "VelocityProfilerDynamic: Input '" + inputName + "' already exists"
            );
        }

        inputs.put(inputName, 0.0);
    }

    public double velocityProfile(String inputName, double input) {
        if (!inputs.containsKey(inputName)) {
            throw new IllegalArgumentException(
                    "VelocityProfilerDynamic: Unknown input name '" + inputName + "'"
            );
        }

        double current = inputs.get(inputName);

        current += (input - current) * smoothingFactor;

        inputs.put(inputName, current);

        return current;
    }


    public void clear() {
        inputs.clear();
    }
}

