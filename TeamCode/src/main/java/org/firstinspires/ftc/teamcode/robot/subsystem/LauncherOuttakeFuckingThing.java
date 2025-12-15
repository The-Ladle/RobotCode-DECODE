package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.util.Range;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.util.Data;

@Config
public class LauncherOuttakeFuckingThing implements Subsystem {
    public static final LauncherOuttakeFuckingThing INSTANCE = new LauncherOuttakeFuckingThing();
    private LauncherOuttakeFuckingThing() {}

    // --- Hardware ---
    private MotorEx motorOne;
    private MotorEx motorTwo;
    private ServoEx turretLatch;
    private ServoEx angleServo;

    // --- Control Flags ---
    // If true: Calculate RPM/Angle from Limelight distance every loop
    // If false: Stay at the last set RPM/Angle (Manual Mode)
    public static boolean autoCalculate = true;

    // --- Encoder info ---
    public static double TICKS_PER_REV = 28.0;

    // --- Geometry constants ---
    private static final double NEUTRAL_HOOD_ANGLE_DEG = 15.0;
    private static final double SERVO_RANGE_DEG        = 355.0;
    private static final double SERVO_PER_HOOD_DEG     = 380.0 / 38.0;

    public static double shooterAngle = 15.0;

    // Turret latch positions
    public static double turret_Closed  = 0.6;
    public static double turret_Open    = 0.07;

    // Preset RPMs
    public static double SLOW_RPM   = 3000;
    public static double LAUNCH_RPM = 3600;

    public static PIDCoefficients COEFFS = new PIDCoefficients(
            0.0017,
            0.000000,
            0.0
    );

    public double angle;
    public static double kS = 0.0175;
    public static double kV = 0.0001815;
    public static double kA = 0.0;

    private ControlSystem velocityPID;
    public static double targetRpm = 0.0;
    private static final double LOOP_DT = 0.02;
    private double lastTargetRpm = 0.0;

    // --- Fallback when Limelight / distance is bad ---
    public static double FALLBACK_RPM   = 2700;
    public static double FALLBACK_ANGLE = 32.0;

    // How long a vision solution is considered "fresh"
    public static double VISION_TIMEOUT_SEC = 0.4;

    // Only use fallback when this is true (so TeleOp can leave it off)
    public static boolean USE_FALLBACK = false;

    private boolean haveFreshVisionSolution = false;
    private double timeSinceLastVision      = 999.0;

    public double getTargetRpm() {
        return targetRpm;
    }

    @Override
    public void initialize() {
        motorOne = new MotorEx("launcher_one");
        motorTwo = new MotorEx("launcher_two");
        turretLatch = new ServoEx("latch_servo");
        angleServo  = new ServoEx("angle_servo");

        velocityPID = ControlSystem.builder()
                .velPid(COEFFS)
                .build();

        setTargetRpm(0.0);
        setAngle(shooterAngle);


        // Default to Auto Calculation on startup
        autoCalculate = false;
        haveFreshVisionSolution = false;
        timeSinceLastVision = 999.0;
    }

    // --- Manual Overrides ---

    /** Stop using Limelight calculation and hold this specific RPM/Angle */
    public void setManualShooter(double rpm, double angleDeg) {
        autoCalculate = false; // Disable auto
        setTargetRpm(rpm);
        setAngle(angleDeg);
    }

    /** Re-enable Limelight distance calculation */
    public void enableAutoCalculation() {
        autoCalculate = true;
    }

    public void setAngle(double angleDeg) {
        shooterAngle = angleDeg;
        double hoodOffsetDeg = shooterAngle - NEUTRAL_HOOD_ANGLE_DEG;
        double servoDeg = hoodOffsetDeg * SERVO_PER_HOOD_DEG;
        double servoPos = servoDeg / SERVO_RANGE_DEG;
        servoPos = Range.clip(servoPos, 0.0, 1.0);
        angleServo.setPosition(servoPos);
    }


    public void setTurretLatch(double pos){
        turretLatch.setPosition(pos);
    }

    public void setTargetRpm(double rpm) {
        targetRpm = rpm;
        velocityPID.setGoal(new KineticState(0.0, targetRpm));
    }

    private double ticksPerSecToRpm(double ticksPerSec) {
        return (ticksPerSec / TICKS_PER_REV) * 60.0;
    }

    public double getCurrentRpm(){
        double ticksPerSec = motorOne.getVelocity();
        return ticksPerSecToRpm(ticksPerSec);
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    // --- Fallback Helpers ---

    /** Do we currently have a good Limelight-based solution? */
    public boolean hasFreshVisionSolution() {
        return autoCalculate && haveFreshVisionSolution && timeSinceLastVision < VISION_TIMEOUT_SEC;
    }

    /**
     * Configure what RPM/angle to fall back to if auto-calculation
     * doesn't have a fresh distance.
     */
    public void setFallback(double rpm, double angleDeg) {
        FALLBACK_RPM   = rpm;
        FALLBACK_ANGLE = angleDeg;

        // If auto is on but we DON'T have a fresh vision solution right now,
        // immediately use the fallback so it spins up early.
        if (autoCalculate && !hasFreshVisionSolution()) {
            setTargetRpm(FALLBACK_RPM);
            setAngle(FALLBACK_ANGLE);
        }
    }

    @Override
    public void periodic() {
        // --- 1. AUTO CALCULATION LOGIC ---
        // Age the last vision solution
        timeSinceLastVision += LOOP_DT;



        if (autoCalculate) {
            // Get Limelight result from Turret (Singleton)
            LLResult result = Turret.INSTANCE.runLimelight();
            double turretAngle = Turret.INSTANCE.getMeasuredAngleDeg();

            // Use the FILTERED Limelight distance (No Odometry)
            double distIn = VisionDistanceHelper.distanceToGoalFromLimelight(result, turretAngle);

            // Only update if we have a valid distance
            if (!Double.isNaN(distIn) && distIn > 0) {
                // We have a valid distance -> mark fresh
                haveFreshVisionSolution = true;
                timeSinceLastVision     = 0.0;

                double firstDist = Data.LAUNCHER_POSES[0][0];
                double lastDist  = Data.LAUNCHER_POSES[Data.LAUNCHER_POSES.length - 1][0];

                // Clamp distance into the calibrated range
                distIn = Range.clip(distIn, firstDist, lastDist);

                double speed;

                int i1 = 1;
                while (i1 < Data.LAUNCHER_POSES.length &&
                        Data.LAUNCHER_POSES[i1][0] < distIn) {
                    i1++;
                }
                int i0 = i1 - 1;

                double d0 = Data.LAUNCHER_POSES[i0][0];
                double d1 = Data.LAUNCHER_POSES[i1][0];

                double t = (Math.abs(d1 - d0) < 1e-6)
                        ? 0.0
                        : (distIn - d0) / (d1 - d0);

                double speed0 = Data.LAUNCHER_POSES[i0][1];
                double speed1 = Data.LAUNCHER_POSES[i1][1];
                double angle0 = Data.LAUNCHER_POSES[i0][2];
                double angle1 = Data.LAUNCHER_POSES[i1][2];

                speed = lerp(speed0, speed1, t);
                angle = lerp(angle0, angle1, t);

                // Set the targets automatically
               setTargetRpm(speed);
               setAngle(angle);
            } else {
                // This loop had no valid distance
                haveFreshVisionSolution = false;

                // If we've gone "too long" without vision, actively fall back
                if (USE_FALLBACK && timeSinceLastVision > VISION_TIMEOUT_SEC) {
                    setTargetRpm(FALLBACK_RPM);
                    setAngle(FALLBACK_ANGLE);
                }
            }
        } else {
            // In pure manual mode: no auto solution
            haveFreshVisionSolution = false;
        }

        // --- 2. MOTOR CONTROL (Runs for both Manual and Auto) ---
        double ticksPerSec = motorOne.getVelocity();
        double measRpm     = ticksPerSecToRpm(ticksPerSec);

        double pidOut = velocityPID.calculate(new KineticState(0.0, measRpm));

        double targetAccelRpmPerSec = (targetRpm - lastTargetRpm) / LOOP_DT;
        lastTargetRpm = targetRpm;

        double ff = 0.0;
        if (Math.abs(targetRpm) > 1e-3) {
            ff += kS * Math.signum(targetRpm);
        }
        ff += kV * targetRpm;
        ff += kA * targetAccelRpmPerSec;

        double power = Range.clip(pidOut + ff, -1.0, 1.0);

        motorOne.setPower(power);
        motorTwo.setPower(power);
    }
}
