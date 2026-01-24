package org.firstinspires.ftc.teamcode.omeN_code.FancyAutos;

import com.pedropathing.geometry.Pose;

public class Poses {
    //Blue Poses
    public static final Pose farStartPose = new Pose(144-88, 9, Math.toRadians(90)); // Start Pose for far side
    public static final Pose farScorePose = new Pose(144-88, 19, Math.toRadians(111)); // Scoring Pose for far side
    public static final Pose lineupFarPose = new Pose(144-89, 41, Math.toRadians(180)); // Farthest set
    public static final Pose gobbleFarPose = new Pose(144-125, 35, Math.toRadians(180)); // Farthest set
    public static final Pose lineupMidPose = new Pose(144-89, 60, Math.toRadians(180)); // Middle set
    public static final Pose gobbleMidPose = new Pose(144-127.5, 54, Math.toRadians(180)); // Middle set
    public static final Pose gateOpenPose = new Pose(144-135, 76, Math.toRadians(180)); //Open da gate
    public static final Pose lineupClosePose = new Pose(144-90, 77, Math.toRadians(180)); // Close set
    public static final Pose gobbleClosePose = new Pose(144-120, 83, Math.toRadians(180)); // Close set
    public static final Pose closeStartPose = new Pose((144-88), 135, Math.toRadians(90)); //Close start Pose
    public static final Pose closeScorePose = new Pose((144-80), 87, Math.toRadians(137)); //Close score pose



    //Red Poses
    public static final Pose farStartPoseR = new Pose(88, 9, Math.toRadians(90)); // Start Pose of our robot.
    public static final Pose farScorePoseR = new Pose(88, 19, Math.toRadians(69)); // Scoring Pose of our robot. It is facing the goal at a 136 degree angle.
    public static final Pose farLineupPoseR = new Pose(89, 44, Math.toRadians(0)); // Farthest (First Set)
    public static final Pose farGobblePoseR = new Pose(125, 34, Math.toRadians(0)); // Farthest (First Set)
    public static final Pose midLineupPoseR = new Pose(89, 70, Math.toRadians(0)); // Middle (Second Set)
    public static final Pose midGobblePoseR = new Pose(130, 64, Math.toRadians(0)); // Middle (Second Set)
    public static final Pose gateOpenPoseR = new Pose(135, 76, Math.toRadians(0));
    public static final Pose closeLineupPoseR = new Pose(90, 84, Math.toRadians(0)); // Closest (Second Set)
    public static final Pose closeGobblePoseR = new Pose(125, 90, Math.toRadians(0)); // Closest (Second Set)
    private static final Pose closeStartPoseR = new Pose(88, 135, Math.toRadians(90)); // Start Pose of our robot.
    private static final Pose closeScorePoseR = new Pose(85, 92, Math.toRadians(40.5));
}
