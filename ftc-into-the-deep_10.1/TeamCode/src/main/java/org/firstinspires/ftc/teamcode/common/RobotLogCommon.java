package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.util.RobotLog;

import java.util.logging.Level;

// This version of the class RobotLogCommon does not use the Java logger;
// instead it passes all logging requests through to the FTC RobotLog.
public class RobotLogCommon {

    private static final String TAG = RobotLogCommon.class.getSimpleName();

    // For ranking use the values from java.util.logging.Level.
    public enum CommonLogLevel {
        off(Level.OFF), e(Level.SEVERE), w(Level.WARNING),
        i(Level.INFO), c(Level.CONFIG),
        d(Level.FINE), v(Level.FINER), vv(Level.FINEST);

        private final Level logLevel;

        CommonLogLevel(Level pLogLevel) {
            logLevel = pLogLevel;
        }

        Level getJavaLogLevel() {
            return logLevel;
        }
    }

    public enum OpenStatus {
        LOGGING_DISABLED, NEW_LOGGER_CREATED,
    }

    public enum LogIdentifier {AUTO_LOG, TELEOP_LOG, TEST_LOG, APP_LOG, FTC_ROBOTLOG, NONE}

    private static CommonLogLevel currentLogLevel = CommonLogLevel.d; // default
    private static CommonLogLevel mostDetailedLogLevel = CommonLogLevel.d; // default

    // Note that the parameter String pLogDirPath is present for compatibility with
    // the Java logger but is not used (and may be null) here.
    public static synchronized OpenStatus initialize(LogIdentifier pIdentifier, String pLogDirPath) {

        RobotLog.dd(TAG, "Request to initialize logger " + pIdentifier);

        if (pIdentifier == LogIdentifier.NONE) {
            RobotLog.dd(TAG, "For log id NONE no logger will be initialized");
            currentLogLevel = CommonLogLevel.off;
            return OpenStatus.LOGGING_DISABLED;
        }

        // All other logs will be directed to the FTC RobotLog.
        RobotLog.dd(TAG, "Logger initialized to " + LogIdentifier.FTC_ROBOTLOG);
        return OpenStatus.NEW_LOGGER_CREATED;
    }

    public static synchronized CommonLogLevel getMostDetailedLogLevel() {
        return mostDetailedLogLevel;
    }

    public static synchronized void setMostDetailedLogLevel(final CommonLogLevel pLogLevel) {
        mostDetailedLogLevel = pLogLevel;
    }

    // Uses the ranking of the Java Level (see the documentation for the integer
    // values) to determine if a requested level is loggable or not.
    public static synchronized boolean isLoggable(final CommonLogLevel pLogLevel) {
        return currentLogLevel != CommonLogLevel.off &&
                pLogLevel.getJavaLogLevel().intValue() >= mostDetailedLogLevel.getJavaLogLevel().intValue();
    }

    // Error
    public static void e(String pTAG, String pLogMessage) {
        RobotLog.ee(pTAG, pLogMessage);
    }

    // Information
    public static void i(String pTAG, String pLogMessage) {
        RobotLog.ii(pTAG, pLogMessage);
    }

    // Configuration
    public static void c(String pTAG, String pLogMessage) { RobotLog.ii(pTAG, pLogMessage); }

    // Debugging
    public static void d(String pTAG, String pLogMessage) {
        RobotLog.dd(pTAG, pLogMessage);
    }

    // Verbose
    public static void v(String pTAG, String pLogMessage) {
        RobotLog.vv(pTAG, pLogMessage);
    }

    // Very verbose
    public static void vv(String pTAG, String pLogMessage) {
        RobotLog.vv(pTAG, pLogMessage);
    }

    // When we're using the FTC RobotLog this call is a no-op.
    public static synchronized void closeLog() {
    }

}
