package org.firstinspires.ftc.teamcode.V1.Autos;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.V1.IntermediateTeleop;
import org.firstinspires.ftc.teamcode.V1.subsystems.ballKickers;
import org.firstinspires.ftc.teamcode.V1.subsystems.colorSensors;
import org.firstinspires.ftc.teamcode.V1.subsystems.distanceSensors;
import org.firstinspires.ftc.teamcode.V1.subsystems.flywheel;
import org.firstinspires.ftc.teamcode.V1.subsystems.intake;
import org.firstinspires.ftc.teamcode.V1.subsystems.limelight;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Far Blue 12")
@Configurable

public class farBlueAuto extends OpMode {

//Scores preload, close preset, middle preset and far preset
    //No indexing or gate opening yet



    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private boolean redAlliance=true;
    private boolean allianceChosen=false;
    private static double scoreHeadingTolerance=0.05;
    private static double scoreTranslationalConstraint=0.5;
    private static double scoreVelocityConstraint=0;
    private final Pose startPose = new Pose(144-88, 9, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(144-88, 19, Math.toRadians(111)); // Scoring Pose of our robot. It is facing the goal at a 136 degree angle.
    private final Pose lineup1Pose = new Pose(144-89, 36, Math.toRadians(180)); // Farthest (First Set)
    private final Pose gobble1Pose = new Pose(144-125, 30, Math.toRadians(180)); // Farthest (First Set)
    private final Pose lineup2Pose = new Pose(144-89, 57, Math.toRadians(180)); // Middle (Second Set)
    private final Pose gobble2Pose = new Pose(144-127.5, 51, Math.toRadians(180)); // Middle (Second Set)
    private final Pose gateOpenPose = new Pose(144-135, 76, Math.toRadians(180));
    private final Pose scorePose2 = new Pose(144-85, 19, Math.toRadians(112));
    private final Pose scorePose3 = new Pose(144-88, 19, Math.toRadians(110.5));
    private final Pose lineup3Pose = new Pose(144-90, 77, Math.toRadians(180)); // Closest (Second Set)
    private final Pose gobble3Pose = new Pose(144-120, 83, Math.toRadians(180)); // Closest (Second Set)
    private final Pose scorePose4 = new Pose(144-88, 19, Math.toRadians(112.5));





    private PathChain scorePreload, grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, park;
    private intake intake;
    private flywheel flywheel;
    private ballKickers ballKickers;
    private limelight limelight;
    private colorSensors colorSensors;
    private distanceSensors distanceSensors;

    private static int launchVel=1320;
    private static double UpRightPos=135;
    private static double UpLeftPos=220;
    private static double DownRightPos=90;
    private static double DownLeftPos=280;
    private static double intakePower=-0.6;
    private static double firstKickWait=0.5;
    private static double thirdKickWait=0.5;
    private static double colorSensorTimeout=2;

    private boolean launch=false;
    private boolean startNextPose=true;
    public void buildPaths() {
            scorePreload = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, scorePose))
                    .setHeadingConstraint(Math.toRadians(scoreHeadingTolerance))
                    .setTranslationalConstraint(scoreTranslationalConstraint)
                    .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                    .setVelocityConstraint(scoreVelocityConstraint)
                    .build();

        /* grabPickup1 PathChain --> lines up for the first set of artifacts, then
          turns on the intake and gobbles them up in a line  */

            grabPickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose, lineup1Pose))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), lineup1Pose.getHeading())
                    //.addTemporalCallback(1, intake_change(1))
                    .addPath(new BezierLine(lineup1Pose, gobble1Pose))
                    .setConstantHeadingInterpolation(lineup1Pose.getHeading())
                    .build();

            /* scorePickup1 PathChain --> moves to the scoring position  */

            scorePickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(gobble1Pose, scorePose2))
                    .setHeadingConstraint(Math.toRadians(scoreHeadingTolerance))
                    .setTranslationalConstraint(scoreTranslationalConstraint)
                    .setLinearHeadingInterpolation(gobble1Pose.getHeading(), scorePose2.getHeading())
                    .setVelocityConstraint(scoreVelocityConstraint)
                    .build();

        /* grabPickup2 PathChain --> lines up for the second set of artifacts, then
           turns on the intake and gobbles them up in a line  */

            grabPickup2 = follower.pathBuilder()

                    .addPath(new BezierLine(scorePose2, lineup2Pose))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), lineup2Pose.getHeading())
                    .addPath(new BezierLine(lineup2Pose, gobble2Pose))
                    .setConstantHeadingInterpolation(lineup2Pose.getHeading())
                    .build();

            scorePickup2 = follower.pathBuilder()
                    .addPath(new BezierLine(lineup2Pose, scorePose3))
                    .setHeadingConstraint(Math.toRadians(scoreHeadingTolerance))
                    .setTranslationalConstraint(scoreTranslationalConstraint)
                    .setLinearHeadingInterpolation(lineup2Pose.getHeading(), scorePose2.getHeading())
                    .setVelocityConstraint(scoreVelocityConstraint)
                    .build();


            grabPickup3 = follower.pathBuilder()

                    .addPath(new BezierLine(scorePose3, lineup3Pose))
                    .setLinearHeadingInterpolation(scorePose2.getHeading(), lineup3Pose.getHeading())
                    .addPath(new BezierLine(lineup3Pose, gobble3Pose))
                    .setConstantHeadingInterpolation(lineup3Pose.getHeading())
                    .build();

            scorePickup3 = follower.pathBuilder()
                    .addPath(new BezierLine(gobble3Pose, scorePose4))
                    .setHeadingConstraint(Math.toRadians(scoreHeadingTolerance))
                    .setTranslationalConstraint(scoreTranslationalConstraint)
                    .setLinearHeadingInterpolation(gobble3Pose.getHeading(), scorePose4.getHeading())
                    .setVelocityConstraint(scoreVelocityConstraint)
                    .build();

            park = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose4, lineup2Pose))
                    .setLinearHeadingInterpolation(scorePose3.getHeading(), gobble1Pose.getHeading())
                    .build();

    }




    public void autonomousPathUpdate() {
        switch (pathState) {

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

//Start flywheel, set speed, go to score pos
            case 0:
                intake.setPower(intakePower);
                intake.update();
                follower.setMaxPower(1);  //slow down the path following if necessary
                follower.followPath(scorePreload, true);
                startNextPose=false;
                setPathState(1);
                break;
//Launch 1st set, go to pickup 2nd
            case 1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {

                    launch=true;
                    launchArtifactsE();
                    if(startNextPose) {
                        follower.followPath(grabPickup1, true);
                        startIntake();
                        setPathState(2);
                    }
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
//                        stopIntake();
//                        follower.followPath(openGate, true);
                    setPathState(3);
                }
                break;
//go to launch 2nd set
            case 3:
                if(!follower.isBusy()) {
                    startIntake();
                    unBlock();
                    follower.followPath(scorePickup1,true);
                    startNextPose=false;
                    setPathState(4);
                }
                break;
//launch 2nd set, go to pickup 3rd set
            case 4:
                if(!follower.isBusy()) {
                    launchArtifactsE();
                    if(startNextPose) {
                        startIntake();
                        follower.followPath(grabPickup2, true);
                        setPathState(5);
                    }
                }
                break;
//go to launch 3rd set
            case 5:
                if(!follower.isBusy()) {
                    unBlock();
                    follower.followPath(scorePickup2,true);
                    setPathState(6);
                }
                break;
//launch 3rd set, go to pickup 4th set
            case 6:
                if(!follower.isBusy()) {
                    launchArtifactsE();
                    startIntake();
                    follower.followPath(grabPickup3,true);
                    setPathState(7);
                }
                break;
//go to launch 4th set
            case 7:
                if(!follower.isBusy()) {
                    unBlock();
                    follower.followPath(scorePickup3,true);
                    setPathState(8);
                }
                break;
//launch 4th set, go to park
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    launchArtifactsE();
                    stopIntake();
                    follower.followPath(park, false);
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()){
                    IntermediateTeleop.startingPose = follower.getPose();
                    setPathState(-1);
                }
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }


    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();

        flywheel.update(launchVel);
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        intake = new intake(hardwareMap );
        limelight = new limelight(hardwareMap);
        flywheel = new flywheel(hardwareMap);
        ballKickers = new ballKickers(hardwareMap);
        colorSensors = new colorSensors(hardwareMap);
        distanceSensors= new distanceSensors(hardwareMap);

        limelight.init();
        limelight.setPipeline(3);
        flywheel.init();
        ballKickers.retractLeft();
        ballKickers.retractRight();
        ballKickers.update();
        intake.setPower(0);

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);

    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}





    public void startIntake(){
        intake.setPower(intakePower);
        intake.update();
//        ballKickers.doubleblock();
//        ballKickers.update();
    }
    public void unBlock(){
        ballKickers.retractLeft();
        ballKickers.retractRight();
    }
    public void stopIntake(){
        intake.setPower(0);
    }
    public void launchArtifactsE() {
        actionTimer.resetTimer();
        while((Math.abs(flywheel.getVelocity()-launchVel)!=0)&&actionTimer.getElapsedTimeSeconds()<firstKickWait){
            flywheel.update(launchVel);
            follower.update();
            telemetry.addData("launchVel", flywheel.getVelocity());
            telemetry.update();
        }
        if(distanceSensors.getSide()==1){ //If efficient side is right
            kickRight();
            kickLeft();
        } else{
            kickLeft();
            kickRight();
        }
        actionTimer.resetTimer();
        while((ballKickers.getRightPos()>DownRightPos)&&(ballKickers.getLeftPos()<DownLeftPos)){
            flywheel.update(launchVel);
            follower.update();
        }
        while((colorSensors.getColorLeft()<1&&colorSensors.getColorRight()<1)&&(actionTimer.getElapsedTimeSeconds()<colorSensorTimeout)){
            flywheel.update(launchVel);
            follower.update();
        }
//        actionTimer.resetTimer();
//        while(actionTimer.getElapsedTimeSeconds()<thirdKickWait){
//            flywheel.update(launchVel);
//        }
        kickBoth();

        startNextPose=true;
    }
    public void kickLeft(){
        while(Math.abs(flywheel.getVelocity()-launchVel)!=0){
            flywheel.update(launchVel);
            follower.update();
            telemetry.addData("launchVel", flywheel.getVelocity());
            telemetry.update();
        }
        ballKickers.kickLeft();
        ballKickers.update();
        while(ballKickers.getLeftPos()>UpLeftPos){
            follower.update();
            flywheel.update(launchVel);
        }
        ballKickers.retractLeft();
        ballKickers.update();
    }
    public void kickRight(){
        while(Math.abs(flywheel.getVelocity()-launchVel)!=0){
            follower.update();
            flywheel.update(launchVel);
            telemetry.addData("launchVel", flywheel.getVelocity());
            telemetry.update();
        }
        ballKickers.kickRight();
        ballKickers.update();
        while(ballKickers.getRightPos()<UpRightPos){
            follower.update();
            flywheel.update(launchVel);
        }
        ballKickers.retractRight();
        ballKickers.update();
    }
    public void kickBoth(){
        while(Math.abs(flywheel.getVelocity()-launchVel)!=0){
            follower.update();
            flywheel.update(launchVel);
        }
        ballKickers.kickRight();
        ballKickers.kickLeft();
        ballKickers.update();
        while((ballKickers.getRightPos()<UpRightPos)||(ballKickers.getLeftPos()>UpLeftPos)){
            follower.update();
            flywheel.update(launchVel);
        }

        ballKickers.retractRight();
        ballKickers.retractLeft();
        ballKickers.update();
    }
}