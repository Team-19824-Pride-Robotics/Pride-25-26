package org.firstinspires.ftc.teamcode.omeN_code.FancyAutos;

import com.pedropathing.geometry.Pose;

public class Poses {
    //Blue Poses
    public static Pose transferPose;
    public static final Pose startFarPose = new Pose(144-88, 9, Math.toRadians(90)); // Start Pose for far side
    public static final Pose scoreFarPose = new Pose(144-88, 19, Math.toRadians(111.5)); // Scoring Pose for far side
    public static final Pose lineupFarthestPose = new Pose(144-134, 12.5, Math.toRadians(180));
    public static final Pose gobbleFarthestPose = new Pose(144-134, 12.5, Math.toRadians(180)); // Farthest set
    public static final Pose lineupFarPose = new Pose(144-95, 45, Math.toRadians(180)); // Farthest set
    public static final Pose gobbleFarPose = new Pose(144-130, 30, Math.toRadians(180)); // Farthest set
    public static final Pose lineupMidPose = new Pose(144-95, 65, Math.toRadians(180)); // Middle set
    public static final Pose gobbleMidPose = new Pose(144-125, 54, Math.toRadians(180)); // Middle set
    public static final Pose gateOpenPose = new Pose(144-124, 70, Math.toRadians(270)); //Open da gate
    public static final Pose lineupClosePose = new Pose(144-95, 80, Math.toRadians(180)); // Close set
    public static final Pose gobbleClosePose = new Pose(144-120, 83, Math.toRadians(180)); // Close set
    public static final Pose startClosePose = new Pose((144-88), 135, Math.toRadians(90)); //Close start Pose
    public static final Pose scoreClosePose = new Pose((144-80), 87, Math.toRadians(150)); //Close score pose
    public static final Pose scoreMidPose = new Pose((144-80), 96, Math.toRadians(145));

    //Red Poses
    public static final Pose startFarPoseR = startFarPose.mirror(); // Start Pose of our robot.
    public static final Pose scoreFarPoseR = scoreFarPose.mirror(); // Scoring Pose of our robot. It is facing the goal at a 136 degree angle.
    public static final Pose lineupFarPoseR = lineupFarPose.mirror(); // Farthest (First Set)
    public static final Pose gobbleFarPoseR = gobbleFarPose.mirror(); // Farthest (First Set)
    public static final Pose lineupMidPoseR = lineupMidPose.mirror(); // Middle (Second Set)
    public static final Pose gobbleMidPoseR = gobbleMidPose.mirror(); // Middle (Second Set)
    public static final Pose gateOpenPoseR = gateOpenPose.mirror();
    public static final Pose lineupClosePoseR = lineupClosePose.mirror(); // Closest (Second Set)
    public static final Pose gobbleClosePoseR = gobbleClosePose.mirror(); // Closest (Second Set)
    public static final Pose startClosePoseR = startClosePose.mirror(); // Start Pose of our robot.
    public static final Pose scoreMidPoseR = scoreMidPose.mirror();
    public static final Pose scoreClosePoseR = scoreClosePose.mirror();
}
