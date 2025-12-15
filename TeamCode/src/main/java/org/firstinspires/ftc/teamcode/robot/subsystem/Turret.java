package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;

@Config
public class Turret implements Subsystem {

    // --- Singleton ---
    public static final Turret INSTANCE = new Turret();

    public Turret() {
    }

    // --- States ---
    public enum TurretState {
        OFF,
        MANUAL,        // angle setpoint from dashboard/driver
        LIMELIGHT,     // pure LL PID on tx -> power
        SNAP_TO_GOAL,  // short angle PID to remembered goal angle
        ODOMETRY_AIM   // New: continuously aim at goal using Pedro Pathing pose
    }

    private TurretState state = TurretState.OFF;

    // --- Hardware ---
    private CRServo turretOne;
    private CRServo turretTwo;
    public AnalogInput turretFeedback;
    private IMUEx imu;
    public Limelight3A limelight;

    // --- LED + readiness ---
    // goBILDA status LED on a servo port (optional)
    private Servo shootStatusLed;

    // LED positions (Dashboard-tunable)
    public static double LED_POS_OFF = 0.00;  // off / idle
    public static double LED_POS_NOT_READY = 0.25;  // e.g. red / amber
    public static double LED_POS_READY = 0.75;  // e.g. green

    // Readiness tolerances (Dashboard-tunable)
    public static double RPM_TOLERANCE = 30.0;  // +/- rpm from target
    public static double TX_SHOOT_TOL_DEG = 1.5;   // tx window to be "aimed"

    private boolean readyForShot = false;

    // --- Analog mapping / geometry ---
    // Tune MIN/MAX_VOLTAGE in Dashboard so measured angle roughly matches reality.
    public static double MIN_VOLTAGE = 3.25;
    public static double MAX_VOLTAGE = 0.05;

    public static double SERVO_RANGE_DEG = 355.0;   // analog span
    public static double TURRET_GEAR_RATIO = 0.815;   // turretDeg = servoDeg * ratio

    public static double MIN_ANGLE_DEG = -145.0;      // turret physical/safe range
    public static double MAX_ANGLE_DEG = 145.0;

    // Soft limit margin so we never command or drive into hard stops
    public static double SOFT_LIMIT_DEG = 135.0;

    public static double LOCK_TX_TOL_DEG = 1.0; // lock on relocalization tolerance

    // Dashboard tester – for MANUAL mode
    public static double angle_tester = 0.0;

    // --- Inner angle PID (used only in MANUAL + SNAP_TO_GOAL) ---
    public static double kP_angle = 0.012;
    public static double kI_angle = 0.0;        // start at 0, add later if needed
    public static double kD_angle = 0.00057;    // if it oscillates a lot, try lowering this

    public static double MAX_TURRET_POWER = 1.0;
    public static double MAX_ANGLE_INTEGRAL = 50.0;

    public static double HOLD_TOL_DEG = 0;   // inner loop "close enough"
    public static double SNAP_TOL_DEG = .5;   // when snapping to remembered angle

    private double turretSetpointDeg = 0.0;
    private double angleIntegral = 0.0;
    private double lastAngleError = 0.0;

    // --- Limelight PID around tx (pure LL control in LIMELIGHT state) ---
    public static double kLL_P = -0.0098;   // sign based on turret direction
    public static double kLL_I = 0.0;
    public static double kLL_D = -0.00045;

    public static double MAX_LL_POWER = 1.0; // clamp turret power in LL mode

    private double llIntegral = 0.0;
    private double llLastError = 0.0;
    private double limelightPower = 0.0;

    private static final double DT_SEC = 0.01; // 20 ms loop

    // --- Angle filtering (simple EMA) ---
    public static double FILTER_ALPHA = 0.6;  // 0..1, higher = smoother

    private double filteredAngleDeg = 0.0;
    private boolean filterInitialized = false;

    // --- Limelight filtering / alignment ---
    public static double TX_FILTER_ALPHA = 0.6;
    public static double LIMELIGHT_X_OFFSET_DEG = 3;
    public static double DEADZONE_DEG = .5;

    public static double filteredTx = 0.0;

    // --- Remembered goal turret angle (robot frame, for debug/telemetry) ---
    private boolean hasGoalTurretAngle = false;
    private double lastGoalTurretAngleDeg = 0.0;

    // --- Remembered goal field heading (field frame, for debug/telemetry) ---
    private boolean hasGoalFieldHeading = false;
    private double lastGoalFieldHeadingDeg = 0.0;

    // --- Snapshot when tag was centered (for snap logic) ---
    //   - lockRobotHeadingDeg: robot heading when tx ≈ 0
    //   - lockTurretAngleDeg:  turret angle when tx ≈ 0
    // Later:
    //   deltaRobotHeading = currentHeading - lockRobotHeadingDeg
    //   desiredTurretDeg  = lockTurretDeg - deltaRobotHeading
    private boolean hasLockSnapshot = false;
    private double lockRobotHeadingDeg = 0.0;
    private double lockTurretAngleDeg = 0.0;

    // --- Lifecycle ---

    @Override
    public void initialize() {

        turretOne = ActiveOpMode.hardwareMap().get(CRServo.class, "turret_one");
        turretTwo = ActiveOpMode.hardwareMap().get(CRServo.class, "turret_two");

        turretOne.setDirection(CRServo.Direction.REVERSE);
        turretTwo.setDirection(CRServo.Direction.REVERSE);

        turretFeedback = ActiveOpMode.hardwareMap().get(AnalogInput.class, "turret");





        imu = new IMUEx("imu", Direction.RIGHT, Direction.FORWARD);
        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
        limelight.start();


        // LED mapping is optional – if the servo isn't present it just won't be used
        try {
            shootStatusLed = ActiveOpMode.hardwareMap().get(Servo.class, "status_led");
        } catch (Exception e) {
            shootStatusLed = null;
        }

        filterInitialized = false;
        llIntegral = 0.0;
        llLastError = 0.0;

        // Start wherever we are
        double startAngle = getMeasuredAngleDeg();
        turretSetpointDeg = startAngle;
        angle_tester = startAngle;
        state = TurretState.MANUAL;


        //  setManualAngle(0);
    }

    // --- Helpers ---

    private static double wrapDeg(double a) {
        while (a > 180) a -= 360;
        while (a < -180) a += 360;
        return a;
    }

    public double getRobotHeadingDeg() {
        // Prefer heading from Pedro/Pinpoint (radians -> degrees)
        try {
            Pose pose = PedroComponent.follower().getPose();
            double headingRad = pose.getHeading();       // Pedro heading (ccw+, radians)
            double headingDeg = Math.toDegrees(headingRad);
            return wrapDeg(headingDeg);
        } catch (Exception e) {
            // Fallback to REV IMU if Pedro/Pinpoint isn't ready for some reason
            return wrapDeg(imu.get().inDeg);
        }
    }

    public LLResult runLimelight() {
        return Turret.INSTANCE.limelight.getLatestResult();
    }

    // Raw analog angle (turret degrees, clipped)
    public double getMeasuredAngleDeg() {

        if (turretFeedback == null) {
            return 0.0;
        }
        double v = turretFeedback.getVoltage();

        double minV = Math.min(MIN_VOLTAGE, MAX_VOLTAGE);
        double maxV = Math.max(MIN_VOLTAGE, MAX_VOLTAGE);

        if (Math.abs(maxV - minV) < 1e-4) {
            return 0.0;
        }

        double norm = (v - minV) / (maxV - minV); // 0..1
        norm = Range.clip(norm, 0.0, 1.0);

        double servoDeg = norm * SERVO_RANGE_DEG - (SERVO_RANGE_DEG / 2.0); // -range/2..+range/2
        double turretDeg = servoDeg * TURRET_GEAR_RATIO;

        return Range.clip(turretDeg, MIN_ANGLE_DEG, MAX_ANGLE_DEG);
    }

    // Simple EMA filter of angle for smoother PID / limits
    private double getFilteredAngleDeg() {
        double raw = getMeasuredAngleDeg();

        if (!filterInitialized) {
            filteredAngleDeg = raw;
            filterInitialized = true;
            return filteredAngleDeg;
        }

        filteredAngleDeg = FILTER_ALPHA * filteredAngleDeg
                + (1.0 - FILTER_ALPHA) * raw;

        return filteredAngleDeg;
    }

    // --- State control API ---

    public void off() {
        state = TurretState.OFF;
        turretOne.setPower(0.0);
        turretTwo.setPower(0.0);
    }

    public void manual() {
        state = TurretState.MANUAL;
    }

    public void enableLimelightAim() {
        state = TurretState.LIMELIGHT;
        filteredTx = 0.0;
        llIntegral = 0.0;
        llLastError = 0.0;
    }

    public void disableLimelightAim() {
        state = TurretState.MANUAL;
        filteredTx = 0.0;
        limelightPower = 0.0;
    }

    public void enableAutoAim(boolean enabled) {
        if (enabled) enableLimelightAim();
        else disableLimelightAim();
    }

    /**
     * Use Pedro Pathing pose to aim at the goal coordinates.
     * Does NOT require seeing the tag, but assumes the robot knows where it is.
     */
    public void enableOdometryAim() {
        state = TurretState.ODOMETRY_AIM;
    }

    // For telemetry if you want
    public String turretStateString() {
        return state.toString();
    }

    // --- Setpoint helpers for angle PID (MANUAL + SNAP) ---

    public void setSetpointDeg(double angleDeg) {
        // Never command outside soft limits
        turretSetpointDeg = Range.clip(angleDeg, -SOFT_LIMIT_DEG, SOFT_LIMIT_DEG);
    }

    public void setManualAngle(double angleDeg) {
        manual();

        // FIX: Update the dashboard variable so periodic() doesn't overwrite us!
        angle_tester = angleDeg;

        setSetpointDeg(angleDeg);
    }

    public void nudgeManual(double deltaDeg) {
        if (state != TurretState.MANUAL) return;
        setSetpointDeg(turretSetpointDeg + deltaDeg);
    }

    // --- Inner angle PID (only used in MANUAL + SNAP_TO_GOAL) ---

    private double anglePidStep(double dtSec, double measuredAngle) {
        double error = turretSetpointDeg - measuredAngle;
        double absError = Math.abs(error);

        if (absError < HOLD_TOL_DEG) {
            angleIntegral = 0.0;
            lastAngleError = 0.0;
            return 0.0;
        }

        angleIntegral += error * dtSec;
        angleIntegral = Range.clip(angleIntegral, -MAX_ANGLE_INTEGRAL, MAX_ANGLE_INTEGRAL);

        double deriv = (error - lastAngleError) / dtSec;
        lastAngleError = error;

        double output = kP_angle * error + kI_angle * angleIntegral + kD_angle * deriv;
        output = Range.clip(output, -MAX_TURRET_POWER, MAX_TURRET_POWER);

        return output;
    }

    // --- Limelight outer loop (pure tx PID -> power in LIMELIGHT state) ---

    public void updateLimelightAim(double dtSec) {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            // No tag: don't drive turret in LL mode
            limelightPower = 0.0;

            return;
        }


        double rawTx = result.getTx();
        double tx = rawTx - LIMELIGHT_X_OFFSET_DEG;

        // Filter tx
        filteredTx = TX_FILTER_ALPHA * filteredTx
                + (1.0 - TX_FILTER_ALPHA) * tx;

        // Deadzone: close enough to center
        if (Math.abs(filteredTx) < DEADZONE_DEG) {
            limelightPower = 0.0;

            double turretAngle = getMeasuredAngleDeg();
            double robotHeading = getRobotHeadingDeg();

            // For telemetry
            lastGoalTurretAngleDeg = turretAngle;
            lastGoalFieldHeadingDeg = wrapDeg(robotHeading + turretAngle);
            hasGoalTurretAngle = true;
            hasGoalFieldHeading = true;

            // Snapshot for snap logic
            lockTurretAngleDeg = turretAngle;      // angle where tag was centered
            lockRobotHeadingDeg = robotHeading;     // robot heading at that moment
            hasLockSnapshot = true;

            // ✅ Relocalize Pedro X/Y, keep heading from odometry/IMU
            VisionDistanceHelper.relocalizePedroFromLimelight(
                    PedroComponent.follower(),
                    result,
                    turretAngle
            );

            return;
        }

        // PID on tx -> turret power
        double error = filteredTx; // positive if tag right of center
        llIntegral += error * dtSec;
        double deriv = (error - llLastError) / dtSec;
        llLastError = error;

        double out = kLL_P * error + kLL_I * llIntegral + kLL_D * deriv;
        limelightPower = Range.clip(out, -MAX_LL_POWER, MAX_LL_POWER);
    }

    // --- Snap-to-remembered-goal logic (FIXED) ---
    public void snapToRememberedGoalAndEnable() {
        // 1) If we currently see a tag, just go straight into LL tracking
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            enableLimelightAim();
            return;
        }

        // 2) No tag – use the stored lock snapshot
        if (!hasLockSnapshot) {
            // we've never actually centered on a tag yet
            return;
        }

        double currentHeading = getRobotHeadingDeg();
        double currentAngle = getMeasuredAngleDeg();

        // How much the robot rotated since we last had the tag centered
        double deltaRobotHeading = wrapDeg(currentHeading - lockRobotHeadingDeg);

        // Calculate the ideal turret angle to maintain field heading
        // If robot turned +30°, turret must turn -30° relative to old angle
        double idealTurretDeg = lockTurretAngleDeg - deltaRobotHeading;

        // Wrap to [-180, 180]
        double wrappedTarget = wrapDeg(idealTurretDeg);

        // FIX 1: Check if the wrapped angle is outside our physical limits
        if (Math.abs(wrappedTarget) > SOFT_LIMIT_DEG) {
            // Try the opposite wrap direction (±360°)
            double altTarget = wrappedTarget > 0
                    ? wrappedTarget - 360
                    : wrappedTarget + 360;

            // Use whichever option is:
            // a) Within physical limits
            // b) Closer to current turret position
            if (Math.abs(altTarget) <= SOFT_LIMIT_DEG) {
                double distWrapped = Math.abs(wrappedTarget - currentAngle);
                double distAlt = Math.abs(altTarget - currentAngle);

                if (distAlt < distWrapped) {
                    wrappedTarget = altTarget;
                }
            }

            // FIX 2: If STILL outside limits, check if robot rotated too much
            if (Math.abs(wrappedTarget) > SOFT_LIMIT_DEG) {
                // Calculate max rotation we can compensate for from current lock position
                double maxCompensation = SOFT_LIMIT_DEG - Math.abs(lockTurretAngleDeg);

                if (Math.abs(deltaRobotHeading) > maxCompensation) {
                    // Robot rotated too much - can't reach target without hitting limits
                    // Fall back to MANUAL at current position
                    state = TurretState.MANUAL;
                    turretSetpointDeg = Range.clip(currentAngle, -SOFT_LIMIT_DEG, SOFT_LIMIT_DEG);
                    angle_tester = turretSetpointDeg;
                    return;
                }

                // Last resort: clamp to nearest limit
                wrappedTarget = Range.clip(wrappedTarget, -SOFT_LIMIT_DEG, SOFT_LIMIT_DEG);
            }
        }

        // FIX 3: Final safety check before setting setpoint
        if (Math.abs(wrappedTarget) > SOFT_LIMIT_DEG) {
            // Emergency abort - something went wrong in the math
            state = TurretState.MANUAL;
            turretSetpointDeg = Range.clip(currentAngle, -SOFT_LIMIT_DEG, SOFT_LIMIT_DEG);
            angle_tester = turretSetpointDeg;
            return;
        }

        setSetpointDeg(wrappedTarget);

        // Enter SNAP_TO_GOAL state (short inner PID move)
        state = TurretState.SNAP_TO_GOAL;

        angleIntegral = 0.0;
        lastAngleError = 0.0;
    }

    // --- Shoot readiness + LED ---

    /**
     * Call this once per loop from TeleOp, passing:
     * - flywheel target RPM
     * - flywheel current RPM
     * - filtered distance to goal (inches)
     * - whether your hood/angle is at its target
     * <p>
     * This does NOT block anything; it just sets readyForShot + LED.
     */

    // --- Main periodic ---
    @Override
    public void periodic() {
        double angleNow = getFilteredAngleDeg(); // filtered turret angle
        double power = 0.0;

        // Get the latest Limelight result ONCE so it's in scope everywhere
        LLResult result = limelight.getLatestResult();


        if (result == null || !result.isValid()) {
            shootStatusLed.setPosition(.28);

        }
        if (result.isValid() && Math.abs(filteredTx) > DEADZONE_DEG) {
            shootStatusLed.setPosition(.388);
        }
        if (Math.abs(filteredTx) < DEADZONE_DEG) {
            shootStatusLed.setPosition(.5);
        }


            switch (state) {
                case OFF:
                    turretOne.setPower(0.0);
                    turretTwo.setPower(0.0);
                    return;

                case MANUAL:
                    // Dashboard: angle_tester
                    setSetpointDeg(angle_tester);
                    power = anglePidStep(DT_SEC, angleNow);
                    break;

                case LIMELIGHT:
                    updateLimelightAim(DT_SEC);
                    power = limelightPower;
                    break;

                case ODOMETRY_AIM:
                    Pose pose = PedroComponent.follower().getPose();
                    if (pose != null) {
                        // 1. Get angle to goal in field coordinates (radians)
                        double goalAngleRad = VisionDistanceHelper.getOdometryAngleToGoalRad(pose);

                        // 2. Get robot heading (radians)
                        double robotHeadingRad = pose.getHeading();

                        // 3. Turret needs to face: Goal Angle - Robot Heading
                        double targetRad = goalAngleRad - robotHeadingRad;

                        // 4. Convert to degrees & wrap
                        double targetDeg = Math.toDegrees(targetRad);
                        setSetpointDeg(wrapDeg(targetDeg));
                    }

                    // Use standard angle PID to get there
                    power = anglePidStep(DT_SEC, angleNow);
                    break;

                case SNAP_TO_GOAL: {
                    // If a tag appears while we're snapping, immediately switch to LL
                    if (result != null && result.isValid()) {
                        enableLimelightAim();
                        updateLimelightAim(DT_SEC);
                        power = limelightPower;
                    } else {
                        // Otherwise, keep snapping toward the remembered angle
                        power = anglePidStep(DT_SEC, angleNow);

                        // Once we're close enough, stop and go back to MANUAL
                        if (Math.abs(turretSetpointDeg - angleNow) < SNAP_TOL_DEG) {
                            power = 0.0;

                            // sync MANUAL target to where we snapped
                            angle_tester = turretSetpointDeg;

                            // state = TurretState.MANUAL;
                        }
                    }
                    break;
                }
            }

            // --- SOFT LIMITS + APPLY POWER ---
            if (state != TurretState.OFF) {

                // Hard stop protection: do not drive further OUT past soft limit
                if (angleNow >= SOFT_LIMIT_DEG && power > 0) {
                    power = 0.0;
                }
                if (angleNow <= -SOFT_LIMIT_DEG && power < 0) {
                    power = 0.0;
                }

                turretOne.setPower(power);
                turretTwo.setPower(power);
            }
        }
    }