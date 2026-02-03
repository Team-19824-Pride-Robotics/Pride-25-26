package org.firstinspires.ftc.teamcode.omeN_code.FancyAutos;

import static org.firstinspires.ftc.teamcode.omeN_code.FancyAutos.AllianceSelection.allianceSelected;
import static org.firstinspires.ftc.teamcode.omeN_code.FancyAutos.Poses.gobbleClosePoseR;
import static org.firstinspires.ftc.teamcode.omeN_code.FancyAutos.Poses.gobbleFarPoseR;
import static org.firstinspires.ftc.teamcode.omeN_code.FancyAutos.Poses.gobbleMidPoseR;
import static org.firstinspires.ftc.teamcode.omeN_code.FancyAutos.Poses.lineupClosePoseR;
import static org.firstinspires.ftc.teamcode.omeN_code.FancyAutos.Poses.lineupFarPoseR;
import static org.firstinspires.ftc.teamcode.omeN_code.FancyAutos.Poses.lineupMidPoseR;
import static org.firstinspires.ftc.teamcode.omeN_code.FancyAutos.Poses.scoreClosePose;
import static org.firstinspires.ftc.teamcode.omeN_code.FancyAutos.Poses.scoreFarPose;
import static org.firstinspires.ftc.teamcode.omeN_code.FancyAutos.Poses.scoreFarPoseR;
import static org.firstinspires.ftc.teamcode.omeN_code.FancyAutos.Poses.startFarPose;
import static org.firstinspires.ftc.teamcode.omeN_code.FancyAutos.Poses.startFarPoseR;
import static org.firstinspires.ftc.teamcode.omeN_code.FancyAutos.Poses.gobbleClosePose;
import static org.firstinspires.ftc.teamcode.omeN_code.FancyAutos.Poses.gobbleFarPose;
import static org.firstinspires.ftc.teamcode.omeN_code.FancyAutos.Poses.gobbleMidPose;
import static org.firstinspires.ftc.teamcode.omeN_code.FancyAutos.Poses.lineupClosePose;
import static org.firstinspires.ftc.teamcode.omeN_code.FancyAutos.Poses.lineupFarPose;
import static org.firstinspires.ftc.teamcode.omeN_code.FancyAutos.Poses.lineupMidPose;
import static org.firstinspires.ftc.teamcode.omeN_code.FancyAutos.AllianceSelection.redAlliance;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.omeN_code.Teleop;
import org.firstinspires.ftc.teamcode.omeN_code.subsystems.ballKickers;
import org.firstinspires.ftc.teamcode.omeN_code.subsystems.colorSensors;
import org.firstinspires.ftc.teamcode.omeN_code.subsystems.distanceSensors;
import org.firstinspires.ftc.teamcode.omeN_code.subsystems.flywheel;
import org.firstinspires.ftc.teamcode.omeN_code.subsystems.intake;
import org.firstinspires.ftc.teamcode.omeN_code.subsystems.limelight;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Far Auto")
@Configurable

public class FarAuto extends OpMode {

//Scores preload, close preset, middle preset and far preset
    //No indexing or gate opening yet



    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer, relocTimer;
    private int pathState;
    private static double scoreHeadingTolerance=0.05;
    private static double scoreTranslationalConstraint=0.5;
    private static double scoreVelocityConstraint=0;

    Poses Poses = new Poses();




    private PathChain scorePreload, grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, park;
    private intake intake;
    private flywheel flywheel;
    private ballKickers ballKickers;
    private limelight limelight;
    private colorSensors colorSensors;
    private distanceSensors distanceSensors;

    private static int launchVel=1340;
    private static double UpRightPos=120;
    private static double UpLeftPos=230;
    private static double DownRightPos=90;
    private static double DownLeftPos=290;
    private static double intakePower=-0.8;
    private static double firstKickWait=0.5;
    private static double thirdKickWait=0.5;
    private static double colorSensorTimeout=2;
    private boolean firstLaunch=true;

    private boolean launch=false;
    private boolean startNextPose=true;
    public void buildPaths() {
        if(!redAlliance) {
            scorePreload = follower.pathBuilder()
                    .addPath(new BezierLine(startFarPose, scoreFarPose))
                    .setHeadingConstraint(Math.toRadians(scoreHeadingTolerance))
                    .setTranslationalConstraint(scoreTranslationalConstraint)
                    .setLinearHeadingInterpolation(startFarPose.getHeading(), scoreFarPose.getHeading())
                    .setVelocityConstraint(scoreVelocityConstraint)
                    .build();

        /* grabPickup1 PathChain --> lines up for the first set of artifacts, then
          turns on the intake and gobbles them up in a line  */

            grabPickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(scoreFarPose, lineupFarPose))
                    .setLinearHeadingInterpolation(scoreFarPose.getHeading(), lineupFarPose.getHeading())
                    //.addTemporalCallback(1, intake_change(1))
                    .addPath(new BezierLine(lineupFarPose, gobbleFarPose))
                    .setConstantHeadingInterpolation(lineupFarPose.getHeading())
                    .build();

            /* scorePickup1 PathChain --> moves to the scoring position  */

            scorePickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(gobbleFarPose, scoreFarPose))
                    .setHeadingConstraint(Math.toRadians(scoreHeadingTolerance))
                    .setTranslationalConstraint(scoreTranslationalConstraint)
                    .setLinearHeadingInterpolation(gobbleFarPose.getHeading(), scoreFarPose.getHeading())
                    .setVelocityConstraint(scoreVelocityConstraint)
                    .build();

        /* grabPickup2 PathChain --> lines up for the second set of artifacts, then
           turns on the intake and gobbles them up in a line  */

            grabPickup2 = follower.pathBuilder()

                    .addPath(new BezierLine(scoreFarPose, lineupMidPose))
                    .setLinearHeadingInterpolation(scoreFarPose.getHeading(), lineupMidPose.getHeading())
                    .addPath(new BezierLine(lineupMidPose, gobbleMidPose))
                    .setConstantHeadingInterpolation(lineupMidPose.getHeading())
                    .build();

            scorePickup2 = follower.pathBuilder()
                    .addPath(new BezierLine(gobbleMidPose, scoreClosePose))
                    .setHeadingConstraint(Math.toRadians(scoreHeadingTolerance))
                    .setTranslationalConstraint(scoreTranslationalConstraint)
                    .setLinearHeadingInterpolation(gobbleMidPose.getHeading(), scoreClosePose.getHeading())
                    .setVelocityConstraint(scoreVelocityConstraint)
                    .build();


            grabPickup3 = follower.pathBuilder()

                    .addPath(new BezierLine(scoreFarPose, lineupClosePose))
                    .setLinearHeadingInterpolation(scoreFarPose.getHeading(), lineupClosePose.getHeading())
                    .addPath(new BezierLine(lineupClosePose, gobbleClosePose))
                    .setConstantHeadingInterpolation(lineupClosePose.getHeading())
                    .build();

            scorePickup3 = follower.pathBuilder()
                    .addPath(new BezierLine(gobbleClosePose, scoreClosePose))
                    .setHeadingConstraint(Math.toRadians(scoreHeadingTolerance))
                    .setTranslationalConstraint(scoreTranslationalConstraint)
                    .setLinearHeadingInterpolation(gobbleClosePose.getHeading(), scoreClosePose.getHeading())
                    .setVelocityConstraint(scoreVelocityConstraint)
                    .build();

            park = follower.pathBuilder()
                    .addPath(new BezierLine(scoreFarPose, lineupMidPose))
                    .setLinearHeadingInterpolation(scoreFarPose.getHeading(), lineupMidPose.getHeading())
                    .build();
        } else{
            scorePreload = follower.pathBuilder()
                    .addPath(new BezierLine(startFarPoseR, scoreFarPoseR))
                    .setHeadingConstraint(Math.toRadians(scoreHeadingTolerance))
                    .setTranslationalConstraint(scoreTranslationalConstraint)
                    .setLinearHeadingInterpolation(startFarPoseR.getHeading(), scoreFarPoseR.getHeading())
                    .setVelocityConstraint(scoreVelocityConstraint)
                    .build();

        /* grabPickup1 PathChain --> lines up for the first set of artifacts, then
          turns on the intake and gobbles them up in a line  */

            grabPickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(scoreFarPoseR, lineupFarPoseR))
                    .setLinearHeadingInterpolation(scoreFarPoseR.getHeading(), lineupFarPoseR.getHeading())
                    //.addTemporalCallback(1, intake_change(1))
                    .addPath(new BezierLine(lineupFarPoseR, gobbleFarPoseR))
                    .setConstantHeadingInterpolation(lineupFarPoseR.getHeading())
                    .build();

            /* scorePickup1 PathChain --> moves to the scoring position  */

            scorePickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(gobbleFarPoseR, scoreFarPoseR))
                    .setHeadingConstraint(Math.toRadians(scoreHeadingTolerance))
                    .setTranslationalConstraint(scoreTranslationalConstraint)
                    .setLinearHeadingInterpolation(gobbleFarPoseR.getHeading(), scoreFarPoseR.getHeading())
                    .setVelocityConstraint(scoreVelocityConstraint)
                    .build();

        /* grabPickup2 PathChain --> lines up for the second set of artifacts, then
           turns on the intake and gobbles them up in a line  */

            grabPickup2 = follower.pathBuilder()

                    .addPath(new BezierLine(scoreFarPoseR, lineupMidPoseR))
                    .setLinearHeadingInterpolation(scoreFarPoseR.getHeading(), lineupMidPoseR.getHeading())
                    .addPath(new BezierLine(lineupMidPoseR, gobbleMidPoseR))
                    .setConstantHeadingInterpolation(lineupMidPoseR.getHeading())
                    .build();

            scorePickup2 = follower.pathBuilder()
                    .addPath(new BezierLine(gobbleMidPoseR, scoreFarPoseR))
                    .setHeadingConstraint(Math.toRadians(scoreHeadingTolerance))
                    .setTranslationalConstraint(scoreTranslationalConstraint)
                    .setLinearHeadingInterpolation(gobbleMidPoseR.getHeading(), scoreFarPoseR.getHeading())
                    .setVelocityConstraint(scoreVelocityConstraint)
                    .build();


            grabPickup3 = follower.pathBuilder()

                    .addPath(new BezierLine(scoreFarPoseR, lineupClosePoseR))
                    .setLinearHeadingInterpolation(scoreFarPoseR.getHeading(), lineupClosePoseR.getHeading())
                    .addPath(new BezierLine(lineupClosePoseR, gobbleClosePoseR))
                    .setConstantHeadingInterpolation(lineupClosePoseR.getHeading())
                    .build();

            scorePickup3 = follower.pathBuilder()
                    .addPath(new BezierLine(gobbleClosePoseR, scoreFarPoseR))
                    .setHeadingConstraint(Math.toRadians(scoreHeadingTolerance))
                    .setTranslationalConstraint(scoreTranslationalConstraint)
                    .setLinearHeadingInterpolation(gobbleClosePoseR.getHeading(), scoreFarPoseR.getHeading())
                    .setVelocityConstraint(scoreVelocityConstraint)
                    .build();

            park = follower.pathBuilder()
                    .addPath(new BezierLine(scoreFarPoseR, lineupMidPoseR))
                    .setLinearHeadingInterpolation(scoreFarPoseR.getHeading(), lineupMidPoseR.getHeading())
                    .build();
        }

    }




    public void autonomousPathUpdate() {
        switch (pathState) {

//Start flywheel, set speed, go to score pos
            case 0:

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
                    reverseIntake();
                    launchArtifactsE();
                    launchVel=1100;
                    startIntake();
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
                    reverseIntake();
                    launchArtifactsE();
                    startIntake();
                    relocalize();
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
                    reverseIntake();
                    launchArtifactsE();
                    startIntake();
                    relocalize();
                    stopIntake();
                    follower.followPath(park, false);
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()){
                    Teleop.startingPose = follower.getPose();
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

        relocalize();

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
        relocTimer = new Timer();
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
        if(redAlliance) {
            follower.setStartingPose(startFarPoseR);
        }else{
            follower.setStartingPose(startFarPose);
        }
        buildPaths();


    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        if(!allianceSelected){
            telemetry.addData("WARNING: ALLIANCE NOT SELECTED\nDEFAULT ALLIANCE IS BLUE","" );
            telemetry.update();
        }
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        relocTimer.resetTimer();
        setPathState(0);

    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
        Teleop.startingPose = follower.getPose();
    }





    public void startIntake(){
        intake.setPower(intakePower);
        intake.update();
    } public void reverseIntake(){
        intake.setPower(-intakePower);
        intake.update();
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
            if(firstLaunch){
                startIntake();
                firstLaunch=false;
            }
            kickLeft();
        } else{
            kickLeft();
            if(firstLaunch){
                startIntake();
                firstLaunch=false;
            }
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
        while((ballKickers.getRightPos()<UpRightPos)&&(ballKickers.getLeftPos()>UpLeftPos)){
            follower.update();
            flywheel.update(launchVel);
        }

        ballKickers.retractRight();
        ballKickers.retractLeft();
        ballKickers.update();
    }
    public void relocalize(){
        if(limelight.isValid()){
            limelight.update(follower.getHeading());
            if (follower.getPose().getY()>72){
                follower.setPose(limelight.relocalize(follower.getHeading()));
            }

        }
    }
}