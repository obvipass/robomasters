package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.*;

public class Logger {

    public enum LoggerMode {
        DETAILED(0),
        CRITICAL(1),
        STATUS(2);

        private final int rank;

        LoggerMode(int rank) {
            this.rank = rank;
        }

        public boolean includes(LoggerMode other) {
            return other.rank >= this.rank;
        }
    }

    private LoggerMode currentMode;
    private final Telemetry telemetry;

    // Persistent logs
    private final List<String> history = new ArrayList<>();

    // Overwriting key-value fields
    private final Map<String, Object> fields = new LinkedHashMap<>();

    // Temporary logs with expiration
    private static class TempLog {
        final String message;
        final long expireTime;

        TempLog(String message, long durationMs) {
            this.message = message;
            this.expireTime = System.currentTimeMillis() + durationMs;
        }

        boolean isExpired() {
            return System.currentTimeMillis() > expireTime;
        }
    }

    private final List<TempLog> tempLogs = new ArrayList<>();

    // Constructors
    public Logger(LoggerMode mode, Telemetry telemetry) {
        this.currentMode = mode;
        this.telemetry = telemetry;
    }

    public Logger(Telemetry telemetry) {
        this.currentMode = LoggerMode.CRITICAL;
        this.telemetry = telemetry;
    }

    public void setMode(LoggerMode mode) {
        this.currentMode = mode;
    }

    // Persistent log
    public void permLog(LoggerMode level, Object... messages) {
        if (!currentMode.includes(level)) return;

        StringBuilder sb = new StringBuilder();
        for (Object msg : messages) sb.append(msg);

        String finalMessage = sb.toString();
        System.out.println(finalMessage);
        history.add(finalMessage);
    }

    // Overwriting key-value log
    public void logData(LoggerMode level, String field, Object data) {
        if (!currentMode.includes(level)) return;

        System.out.println(field + ": " + data);

        // store current value (overwrites old)
        fields.put(field, data);
    }

    public void logData(String field, Object data) {
        if (!currentMode.includes(LoggerMode.STATUS)) return;

        System.out.println(field + ": " + data);

        // store current value (overwrites old)
        fields.put(field, data);
    }

    // Temporary log with duration in milliseconds
    public void tempLog(LoggerMode level, double durationSeconds, Object... messages) {
        if (!currentMode.includes(level)) return;

        StringBuilder sb = new StringBuilder();
        for (Object msg : messages) sb.append(msg);

        String finalMessage = sb.toString();
        System.out.println(finalMessage);

        // convert seconds to milliseconds for expireTime
        long durationMs = (long)(durationSeconds * 1000);
        tempLogs.add(new TempLog(finalMessage, durationMs));
    }


    // Update telemetry display
    public void update() {
        // full clear
        telemetry.clearAll();

        // Remove expired temporary logs
        tempLogs.removeIf(TempLog::isExpired);

        // Render persistent logs at the top
        for (String line : history) {
            telemetry.addLine(line);
        }

        // Render overwriting data below perm logs
        for (Map.Entry<String, Object> e : fields.entrySet()) {
            telemetry.addData(e.getKey(), e.getValue());
        }

        // Render temporary logs last
        for (TempLog tl : tempLogs) {
            telemetry.addLine(tl.message);
        }


        telemetry.update();
    }

    // Clear all logs
    public void clear() {
        history.clear();
        fields.clear();
        tempLogs.clear();
        telemetry.clearAll();
        telemetry.update();
    }
}
