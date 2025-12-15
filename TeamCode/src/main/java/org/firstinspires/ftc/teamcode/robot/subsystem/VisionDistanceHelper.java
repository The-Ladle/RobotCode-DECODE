package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Config
public class VisionDistanceHelper {

    // Pedro coords of the GOAL tag (inches) – tune these once it's roughly right
    public static double GOAL_TAG_X_IN = 144- 127.64;
    public static double GOAL_TAG_Y_IN = 130.37;

    // Distance from robot center (turret pivot) to Limelight lens in inches
    public static double CAMERA_RADIUS_IN = 4.704;

    // Pedro field center (tiles are 144x144 in)
    public static final double FIELD_CENTER_X = 72.0;
    public static final double FIELD_CENTER_Y = 72.0;

    // --- Pose filter (Limelight -> Pedro) ---

    // NEW FLAG: Set to false in Dashboard (or code) to stop Pedro from being updated
    public static boolean RELOCALIZATION_ENABLED = true;

    /** 0..1, higher = trust Pedro/odometry more, lower = snap more to Limelight */
    public static double POSE_FILTER_ALPHA = 0.6;

    // --- Distance filter + shooting window (Dashboard-tunable) ---
    /** 0..1, higher = smoother (slower to react) */
    public static double DIST_FILTER_ALPHA = 0.6;
    /** Max jump allowed per update; 0 = unlimited */
    public static double DIST_MAX_JUMP_IN = 10.0;

    /** Min and max distance where a shot is considered "in range" (inches) */
    public static double MIN_SHOOT_DIST_IN = 20.0;
    public static double MAX_SHOOT_DIST_IN = 90.0;

    // Internal filtered distance state
    private static double filteredDistanceIn = Double.NaN;

    /**
     * Relocalize Pedro using a *filtered* Limelight pose:
     *
     * 1) Compute robot pose from Limelight (robotPoseFromLimelight).
     * 2) Blend that XY into Pedro's pose with an exponential filter.
     * 3) Keep heading from Pedro/IMU (odoPose.getHeading()).
     *
     * After this, ALL other code should just use follower.getPose().
     */
    public static void relocalizePedroFromLimelight(
            Follower follower,
            LLResult result,
            double turretAngleDeg
    ) {
        // 1. Check the flag immediately. If false, do nothing.
        if (!RELOCALIZATION_ENABLED) return;

        if (follower == null) return;

        Pose llRobotPedro = robotPoseFromLimelight(result, turretAngleDeg);
        if (llRobotPedro == null) return;

        Pose odoPose = follower.getPose();

        // If Pedro has no pose yet, just seed it with the Limelight pose
        if (odoPose == null) {
            follower.setPose(llRobotPedro);
            return;
        }

        // Clamp alpha into [0, 1]
        double a = POSE_FILTER_ALPHA;
        if (a < 0.0) a = 0.0;
        if (a > 1.0) a = 1.0;

        // Filter XY: newXY = a * oldXY + (1-a) * llXY
        double newX = a * odoPose.getX() + (1.0 - a) * llRobotPedro.getX();
        double newY = a * odoPose.getY() + (1.0 - a) * llRobotPedro.getY();

        // Keep heading from Pedro/IMU so we don't fight your gyro/pinpoint
        double newHeading = odoPose.getHeading();

        Pose filteredPose = new Pose(newX, newY, newHeading);
        follower.setPose(filteredPose);
    }

    /**
     * MT1 + turret:
     * - use botpose as CAMERA pose in FMap/FTC center-origin
     * - map to Pedro coords explicitly
     * - use turret angle + CAMERA_RADIUS_IN to get ROBOT CENTER
     */
    public static Pose robotPoseFromLimelight(LLResult result, double turretAngleDeg) {
        if (result == null || !result.isValid()) {
            return null;
        }

        Pose3D camPose3D = result.getBotpose();  // MT1
        if (camPose3D == null) {
            return null;
        }

        // === 1) Raw FTC coords (center origin), meters ===
        double llX_m = camPose3D.getPosition().x;
        double llY_m = camPose3D.getPosition().y;
        double llYaw_rad = camPose3D.getOrientation().getYaw(AngleUnit.RADIANS);

        // === 2) Map to "field X/Y" like you had before ===
        //   fieldX = inches(LL_y)
        //   fieldY = -inches(LL_x)
        double fieldX = DistanceUnit.INCH.fromMeters(llY_m);   // left/right
        double fieldY = -DistanceUnit.INCH.fromMeters(llX_m);  // up/down field

        // === 3) Center-origin -> Pedro 0..144, 0..144 ===
        double camX_pedro = FIELD_CENTER_X + fieldX;
        double camY_pedro = FIELD_CENTER_Y + fieldY;

        // Heading: LL’s yaw is already CCW+, treat it as camera heading.
        double thetaCam = llYaw_rad;

        // === 4) Turret trig: camera -> robot center ===
        double turretAngleRad = Math.toRadians(turretAngleDeg);  // turret angle relative to robot

        // Robot heading = camera heading - turret angle
        double thetaRobot = thetaCam - turretAngleRad;

        // Robot center is CAMERA minus offset along camera heading
        double xRobot = camX_pedro - CAMERA_RADIUS_IN * Math.cos(thetaCam);
        double yRobot = camY_pedro - CAMERA_RADIUS_IN * Math.sin(thetaCam);

        return new Pose(xRobot, yRobot, thetaRobot);
    }

    // --- Distance helpers ---

    public static double distanceToGoalInches(Pose robotPose) {
        if (robotPose == null) return Double.NaN;

        double dx = GOAL_TAG_X_IN - robotPose.getX();
        double dy = GOAL_TAG_Y_IN - robotPose.getY();
        return Math.hypot(dx, dy);
    }

    /** Raw distance (no filtering) straight from Limelight + geometry. */
    public static double distanceToGoalFromLimelight(LLResult result, double turretAngleDeg) {
        Pose robotPosePedro = robotPoseFromLimelight(result, turretAngleDeg);
        return distanceToGoalInches(robotPosePedro);
    }

    /**
     * Calculates the field-centric angle (in Radians) the robot/turret needs to face
     * to aim at the goal, based purely on Odometry/Pedro pose.
     * Useful for aiming when the Limelight doesn't see a tag.
     */
    public static double getOdometryAngleToGoalRad(Pose robotPose) {
        if (robotPose == null) return 0.0;
        double dx = GOAL_TAG_X_IN - robotPose.getX();
        double dy = GOAL_TAG_Y_IN - robotPose.getY();
        return Math.atan2(dy, dx);
    }

    /**
     * Filtered distance using an exponential moving average plus optional jump clamp.
     * You *can* still use this directly if you want pure-LL distance,
     * but the preferred way now is:
     * distanceToGoalInches(follower.getPose())
     */
    public static double filteredDistanceToGoalFromLimelight(LLResult result, double turretAngleDeg) {
        double raw = distanceToGoalFromLimelight(result, turretAngleDeg);

        // If we have no valid measurement, just return the last filtered value
        if (Double.isNaN(raw) || raw <= 0) {
            return filteredDistanceIn;
        }

        // First valid reading initializes the filter
        if (Double.isNaN(filteredDistanceIn)) {
            filteredDistanceIn = raw;
            return filteredDistanceIn;
        }

        // Optional jump clamp to ignore crazy spikes
        double diff = raw - filteredDistanceIn;
        if (DIST_MAX_JUMP_IN > 0 && Math.abs(diff) > DIST_MAX_JUMP_IN) {
            raw = filteredDistanceIn + Math.signum(diff) * DIST_MAX_JUMP_IN;
        }

        // EMA: newFiltered = alpha * old + (1-alpha) * raw
        filteredDistanceIn = DIST_FILTER_ALPHA * filteredDistanceIn
                + (1.0 - DIST_FILTER_ALPHA) * raw;

        return filteredDistanceIn;
    }

    /** True if distance is in the "we can probably make this shot" window. */
    public static boolean isDistanceInRangeForShot(double distanceIn) {
        if (Double.isNaN(distanceIn) || distanceIn <= 0) return false;
        return distanceIn >= MIN_SHOOT_DIST_IN && distanceIn <= MAX_SHOOT_DIST_IN;
    }
}