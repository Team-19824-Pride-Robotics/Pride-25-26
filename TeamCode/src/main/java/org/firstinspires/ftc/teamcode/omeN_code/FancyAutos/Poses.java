package org.firstinspires.ftc.teamcode.omeN_code.FancyAutos;

import com.pedropathing.geometry.Pose;

public class Poses {
    //Blue Poses
    public static Pose transferPose;
    public static final Pose startFarPose = new Pose(144-88, 9, Math.toRadians(90)); // Start Pose for far side
    public static final Pose scoreFarPose = new Pose(144-88, 19, Math.toRadians(112)); // Scoring Pose for far side
    public static final Pose lineupFarthestPose = new Pose(144-120, 15, Math.toRadians(200));
    public static final Pose gobbleFarthestPose = new Pose(144-131, 14, Math.toRadians(200)); // Farthest set
    public static final Pose lineupFarPose = new Pose(144-95, 42.5, Math.toRadians(180)); // Farthest set
    public static final Pose gobbleFarPose = new Pose(144-135, 32.5, Math.toRadians(180)); // Farthest set
    public static final Pose lineupMidPose = new Pose(144-90, 65, Math.toRadians(180)); // Middle set
    public static final Pose gobbleMidPose = new Pose(144-125, 57.5, Math.toRadians(180)); // Middle set
    public static final Pose gateOpenPose = new Pose(144-125, 70, Math.toRadians(270)); //Open da gate
    public static final Pose lineupClosePose = new Pose(144-90, 80, Math.toRadians(180)); // Close set
    public static final Pose gobbleClosePose = new Pose(144-125, 83, Math.toRadians(180)); // Close set
    public static final Pose startClosePose = new Pose((144-88), 135, Math.toRadians(90)); //Close start Pose
    public static final Pose scoreClosePose = new Pose((144-80), 87, Math.toRadians(150)); //Close score pose
    public static final Pose scoreMidPose = new Pose((144-80), 96, Math.toRadians(145));

    //Red Poses
    public static final Pose startFarPoseR = new Pose(88, 9, Math.toRadians(180-90)); // Start Pose for far side
    public static final Pose scoreFarPoseR = new Pose(88, 19, Math.toRadians(180-112)); // Scoring Pose for far side
    public static final Pose lineupFarthestPoseR = new Pose(120, 15, Math.toRadians(180-200));
    public static final Pose gobbleFarthestPoseR = new Pose(131, 14, Math.toRadians(180-200)); // Farthest set
    public static final Pose lineupFarPoseR = new Pose(95, 42.5, Math.toRadians(180-180)); // Farthest set
    public static final Pose gobbleFarPoseR = new Pose(135, 32.5, Math.toRadians(180-180)); // Farthest set
    public static final Pose lineupMidPoseR = new Pose(100, 65, Math.toRadians(180-180)); // Middle set
    public static final Pose gobbleMidPoseR = new Pose(125, 57.5, Math.toRadians(180-180)); // Middle set
    public static final Pose gateOpenPoseR = new Pose(125, 70, Math.toRadians(180-270)); //Open da gate
    public static final Pose lineupClosePoseR = new Pose(90, 80, Math.toRadians(180-180)); // Close set
    public static final Pose gobbleClosePoseR = new Pose(130, 83, Math.toRadians(180-180)); // Close set
    public static final Pose startClosePoseR = new Pose((88), 135, Math.toRadians(180-90)); //Close start Pose
    public static final Pose scoreClosePoseR = new Pose((80), 87, Math.toRadians(180-150)); //Close score pose
    public static final Pose scoreMidPoseR = new Pose((80), 96, Math.toRadians(180-145));
}
