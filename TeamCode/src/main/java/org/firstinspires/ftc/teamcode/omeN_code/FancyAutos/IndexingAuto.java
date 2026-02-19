package org.firstinspires.ftc.teamcode.omeN_code.FancyAutos;

import static org.firstinspires.ftc.teamcode.omeN_code.FancyAutos.AllianceSelection.allianceSelected;
import static org.firstinspires.ftc.teamcode.omeN_code.FancyAutos.AllianceSelection.redAlliance;
import static org.firstinspires.ftc.teamcode.omeN_code.FancyAutos.Poses.gateOpenPose;
import static org.firstinspires.ftc.teamcode.omeN_code.FancyAutos.Poses.gateOpenPoseR;
import static org.firstinspires.ftc.teamcode.omeN_code.FancyAutos.Poses.gobbleClosePose;
import static org.firstinspires.ftc.teamcode.omeN_code.FancyAutos.Poses.gobbleClosePoseR;
import static org.firstinspires.ftc.teamcode.omeN_code.FancyAutos.Poses.gobbleFarPose;
import static org.firstinspires.ftc.teamcode.omeN_code.FancyAutos.Poses.gobbleFarPoseR;
import static org.firstinspires.ftc.teamcode.omeN_code.FancyAutos.Poses.gobbleMidPose;
import static org.firstinspires.ftc.teamcode.omeN_code.FancyAutos.Poses.gobbleMidPoseR;
import static org.firstinspires.ftc.teamcode.omeN_code.FancyAutos.Poses.lineupClosePose;
import static org.firstinspires.ftc.teamcode.omeN_code.FancyAutos.Poses.lineupClosePoseR;
import static org.firstinspires.ftc.teamcode.omeN_code.FancyAutos.Poses.lineupFarPose;
import static org.firstinspires.ftc.teamcode.omeN_code.FancyAutos.Poses.lineupFarPoseR;
import static org.firstinspires.ftc.teamcode.omeN_code.FancyAutos.Poses.lineupMidPose;
import static org.firstinspires.ftc.teamcode.omeN_code.FancyAutos.Poses.lineupMidPoseR;
import static org.firstinspires.ftc.teamcode.omeN_code.FancyAutos.Poses.scoreFarPose;
import static org.firstinspires.ftc.teamcode.omeN_code.FancyAutos.Poses.scoreFarPoseR;
import static org.firstinspires.ftc.teamcode.omeN_code.FancyAutos.Poses.scoreMidPose;
import static org.firstinspires.ftc.teamcode.omeN_code.FancyAutos.Poses.scoreMidPoseR;
import static org.firstinspires.ftc.teamcode.omeN_code.FancyAutos.Poses.startFarPose;
import static org.firstinspires.ftc.teamcode.omeN_code.FancyAutos.Poses.startFarPoseR;

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

@Autonomous(name = "I Auto")
@Configurable

public class IndexingAuto extends OpMode {

//Scores preload, close preset, middle preset and far preset
    //No indexing or gate opening yet



    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer, relocTimer, loopTimer;
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

    private static int closeVel=1140;
    private static int farVel=1400;
    private static int ejectVel=600;
    private static int farTol=0;
    private static int closeTol=20;
    private static int motif=-1;
    private int launchVel=farVel;
    private int launchTol;
    private static double UpRightPos=130;
    private static double UpLeftPos=236;
    private static double DownRightPos=92;
    private static double DownLeftPos=285;
    private static double intakePower=-1;
    private static double firstKickWait=0.5;
    private static double thirdKickWait=0.5;
    private static double indexWait=1;
    private static double rollWait=3;
    private static double colorSensorTimeout=2;
    private static double gateOpenWait=0.5;
    private boolean firstLaunch=true;

    private boolean eject =false;
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
                    .addPath(new BezierLine(scoreFarPose, lineupMidPose))
                    .setLinearHeadingInterpolation(scoreFarPose.getHeading(), lineupMidPose.getHeading())
                    //.addTemporalCallback(1, intake_change(1))
                    .addPath(new BezierLine(lineupMidPose, gobbleMidPose))
                    .setConstantHeadingInterpolation(lineupMidPose.getHeading())
                    .addPath(new BezierLine(gobbleMidPose, gateOpenPose))
                    .setLinearHeadingInterpolation(gobbleMidPose.getHeading(), gateOpenPose.getHeading())
                    .build();

            /* scorePickup1 PathChain --> moves to the scoring position  */

            scorePickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(gateOpenPose, lineupMidPose))
                    .setLinearHeadingInterpolation(gateOpenPose.getHeading(), lineupMidPose.getHeading())
                    .addPath(new BezierLine(lineupMidPose, scoreMidPose))
                    .setHeadingConstraint(Math.toRadians(scoreHeadingTolerance))
                    .setTranslationalConstraint(scoreTranslationalConstraint)
                    .setLinearHeadingInterpolation(lineupMidPose.getHeading(), scoreMidPose.getHeading())
                    .setVelocityConstraint(scoreVelocityConstraint)
                    .build();

        /* grabPickup2 PathChain --> lines up for the second set of artifacts, then
           turns on the intake and gobbles them up in a line  */

            grabPickup2 = follower.pathBuilder()

                    .addPath(new BezierLine(scoreMidPose, lineupClosePose))
                    .setLinearHeadingInterpolation(scoreMidPose.getHeading(), lineupClosePose.getHeading())
                    .addPath(new BezierLine(lineupClosePose, gobbleClosePose))
                    .setConstantHeadingInterpolation(lineupClosePose.getHeading())
                    .build();

            scorePickup2 = follower.pathBuilder()
                    .addPath(new BezierLine(gobbleClosePose, scoreMidPose))
                    .setHeadingConstraint(Math.toRadians(scoreHeadingTolerance))
                    .setTranslationalConstraint(scoreTranslationalConstraint)
                    .setLinearHeadingInterpolation(gobbleClosePose.getHeading(), scoreMidPose.getHeading())
                    .setVelocityConstraint(scoreVelocityConstraint)
                    .build();


            grabPickup3 = follower.pathBuilder()

                    .addPath(new BezierLine(scoreMidPose, lineupFarPose))
                    .setLinearHeadingInterpolation(scoreMidPose.getHeading(), lineupFarPose.getHeading())
                    .addPath(new BezierLine(lineupFarPose, gobbleFarPose))
                    .setConstantHeadingInterpolation(lineupFarPose.getHeading())
                    .build();

            scorePickup3 = follower.pathBuilder()
                    .addPath(new BezierLine(gobbleFarPose, scoreMidPose))
                    .setHeadingConstraint(Math.toRadians(scoreHeadingTolerance))
                    .setTranslationalConstraint(scoreTranslationalConstraint)
                    .setLinearHeadingInterpolation(gobbleFarPose.getHeading(), scoreMidPose.getHeading())
                    .setVelocityConstraint(scoreVelocityConstraint)
                    .build();

            park = follower.pathBuilder()
                    .addPath(new BezierLine(scoreMidPose, lineupMidPose))
                    .setLinearHeadingInterpolation(scoreMidPose.getHeading(), lineupMidPose.getHeading())
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
                    .addPath(new BezierLine(scoreFarPoseR, lineupMidPoseR))
                    .setLinearHeadingInterpolation(scoreFarPoseR.getHeading(), lineupMidPoseR.getHeading())
                    //.addTemporalCallback(1, intake_change(1))
                    .addPath(new BezierLine(lineupMidPoseR, gobbleMidPoseR))
                    .setConstantHeadingInterpolation(lineupMidPoseR.getHeading())
                    .addPath(new BezierLine(gobbleMidPoseR, gateOpenPoseR))
                    .setLinearHeadingInterpolation(gobbleMidPoseR.getHeading(), gateOpenPoseR.getHeading())
                    .build();

            /* scorePickup1 PathChain --> moves to the scoring position  */

            scorePickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(gateOpenPoseR, lineupMidPoseR))
                    .setLinearHeadingInterpolation(gateOpenPoseR.getHeading(), lineupMidPoseR.getHeading())
                    .addPath(new BezierLine(lineupMidPoseR, scoreMidPoseR))
                    .setHeadingConstraint(Math.toRadians(scoreHeadingTolerance))
                    .setTranslationalConstraint(scoreTranslationalConstraint)
                    .setLinearHeadingInterpolation(lineupMidPoseR.getHeading(), scoreMidPoseR.getHeading())
                    .setVelocityConstraint(scoreVelocityConstraint)
                    .build();

        /* grabPickup2 PathChain --> lines up for the second set of artifacts, then
           turns on the intake and gobbles them up in a line  */

            grabPickup2 = follower.pathBuilder()

                    .addPath(new BezierLine(scoreMidPoseR, lineupClosePoseR))
                    .setLinearHeadingInterpolation(scoreMidPoseR.getHeading(), lineupClosePoseR.getHeading())
                    .addPath(new BezierLine(lineupClosePoseR, gobbleClosePoseR))
                    .setConstantHeadingInterpolation(lineupClosePoseR.getHeading())
                    .build();

            scorePickup2 = follower.pathBuilder()
                    .addPath(new BezierLine(gobbleClosePoseR, scoreMidPoseR))
                    .setHeadingConstraint(Math.toRadians(scoreHeadingTolerance))
                    .setTranslationalConstraint(scoreTranslationalConstraint)
                    .setLinearHeadingInterpolation(gobbleClosePoseR.getHeading(), scoreMidPoseR.getHeading())
                    .setVelocityConstraint(scoreVelocityConstraint)
                    .build();


            grabPickup3 = follower.pathBuilder()

                    .addPath(new BezierLine(scoreMidPoseR, lineupFarPoseR))
                    .setLinearHeadingInterpolation(scoreMidPoseR.getHeading(), lineupFarPoseR.getHeading())
                    .addPath(new BezierLine(lineupFarPoseR, gobbleFarPoseR))
                    .setConstantHeadingInterpolation(lineupFarPoseR.getHeading())
                    .build();

            scorePickup3 = follower.pathBuilder()
                    .addPath(new BezierLine(gobbleFarPoseR, scoreFarPoseR))
                    .setHeadingConstraint(Math.toRadians(scoreHeadingTolerance))
                    .setTranslationalConstraint(scoreTranslationalConstraint)
                    .setLinearHeadingInterpolation(gobbleFarPoseR.getHeading(), scoreFarPoseR.getHeading())
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
                motif = limelight.scanAuto();
                follower.setMaxPower(1);
                switchToFar();
                follower.followPath(scorePreload, true);
                startNextPose=false;
                setPathState(1);
                break;
//Launch 1st set, go to pickup 2nd
            case 1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    launchArtifactsI();
                    launchVel=ejectVel;
                    if(startNextPose) {
                        follower.followPath(grabPickup1, true);
                        startIntake();
                        setPathState(2);
                    }
                }
                break;
                //Open gate, go to launch 2nd set
            case 2:
                if(!follower.isBusy()) {
                    switchToClose();
                    actionTimer.resetTimer();
                    while(actionTimer.getElapsedTimeSeconds()<gateOpenWait){
                        flywheel.update(launchVel);
                        follower.update();
                    }
                    checkOverflow();
                    setPathState(3);
                }
                break;
//go to launch 2nd set
            case 3:
                if(!follower.isBusy()) {
                    startIntake();
                    follower.followPath(scorePickup1,true);
                    startNextPose=false;
                    setPathState(4);
                }
                break;
//launch 2nd set, go to pickup 3rd set
            case 4:
                if(!follower.isBusy()) {
                    reverseIntake();
                    launchArtifactsI();
                    launchVel=ejectVel;
                    startIntake();
                    if(startNextPose) {
                        startIntake();
                        follower.followPath(grabPickup2, true);
                        eject();
                        setPathState(5);
                    }
                }
                break;
//go to launch 3rd set
            case 5:
                if(!follower.isBusy()) {
                    checkOverflow();
                    switchToClose();
                    follower.followPath(scorePickup2,true);
                    setPathState(6);
                }
                break;
//launch 3rd set, go to pickup 4th set
            case 6:
                if(!follower.isBusy()) {
                    reverseIntake();
                    launchArtifactsI();
                    launchVel=ejectVel;
                    startIntake();
                    follower.followPath(grabPickup3,true);
                    eject();
                    setPathState(7);
                }
                break;
//go to launch 4th set
            case 7:
                if(!follower.isBusy()) {
                    checkOverflow();
                    switchToClose();
                    follower.followPath(scorePickup3,true);
                    setPathState(8);
                }
                break;
//launch 4th set, go to park
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    reverseIntake();
                    launchArtifactsI();
                    startIntake();
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
loopTimer.resetTimer();
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();

        flywheel.update(launchVel);
        autonomousPathUpdate();


        // Feedback to Driver Hub for debugging
        telemetry.addData("loop time: ", loopTimer.getElapsedTime());
        telemetry.addData("Motif ", motif);
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
        loopTimer = new Timer();
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




    public void switchToFar(){
        launchVel=farVel;
        launchTol=farTol;
    }
    public void switchToClose(){
        launchVel=closeVel;
        launchTol=closeTol;
    }
    public void startIntake(){
        intake.setPower(intakePower);
        intake.update();
    } public void reverseIntake(){
        intake.setPower(-intakePower);
        intake.update();
    }
    public void stopIntake(){
        intake.setPower(0);
    }
    public void checkOverflow(){
        if(distanceSensors.getCount()>1){
            ballKickers.kickRight();
            ballKickers.update();
            while(ballKickers.getRightPos()<UpRightPos){
                follower.update();
                flywheel.update(launchVel);
            }
            ballKickers.retractRight();
            ballKickers.update();
        }
    }
    public void eject(){
        if(eject){
            ballKickers.kickBoth();
            ballKickers.update();
            while(ballKickers.getRightPos()<UpRightPos){
                follower.update();
                flywheel.update(launchVel);
            }
            ballKickers.retractBoth();
            ballKickers.update();
            eject=false;
        }
    }
    public void launchArtifactsE() {
        actionTimer.resetTimer();
        while((Math.abs(flywheel.getVelocity()-launchVel)>launchTol)||actionTimer.getElapsedTimeSeconds()<firstKickWait){
            flywheel.update(launchVel);
            follower.update();
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
        while((ballKickers.getRightPos()>DownRightPos)||(ballKickers.getLeftPos()<DownLeftPos)){
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
        if(colorSensors.getColorLeft()<1&&colorSensors.getColorRight()<1){
            kickLeft();
            eject=true;
        }else {
            kickBoth();
        }
        actionTimer.resetTimer();
        while(actionTimer.getElapsedTimeSeconds()<thirdKickWait){
            flywheel.update(launchVel);
            follower.update();
        }
        startNextPose=true;
    }
    public void launchArtifactsI(){
        if(motif==-1){
            launchArtifactsE();
        } else{
            actionTimer.resetTimer();
            while((Math.abs(flywheel.getVelocity()-launchVel)!=0)&&actionTimer.getElapsedTimeSeconds()<firstKickWait){
                flywheel.update(launchVel);
                telemetry.addData("launchVel", flywheel.getVelocity());
                telemetry.update();
                follower.update();
            }
            int leftColor=colorSensors.getColorLeft();
            int rightColor=colorSensors.getColorRight();
            if(motif==0){
                rollWait();
                chooseI(2, leftColor, rightColor);
            }else{
                rollWait();
                chooseI(1, leftColor, rightColor);
            }
            startIntake();
            actionTimer.resetTimer();
            while(actionTimer.getElapsedTimeSeconds()<indexWait){
                flywheel.update(launchVel);
                follower.update();
                telemetry.addData("launchVel", flywheel.getVelocity());
                telemetry.update();
            }
            leftColor=colorSensors.getColorLeft();
            rightColor=colorSensors.getColorRight();
            if(motif==1){
                rollWait();
                chooseI(2, leftColor, rightColor);
            }else{
                rollWait();
                chooseI(1, leftColor, rightColor);
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
            while(actionTimer.getElapsedTimeSeconds()<indexWait){
                flywheel.update(launchVel);
                follower.update();
                telemetry.addData("launchVel", flywheel.getVelocity());
                telemetry.update();
            }
            kickBoth();
        }
        startNextPose=true;
    }
    public void rollWait(){
        actionTimer.resetTimer();
        while(actionTimer.getElapsedTimeSeconds()<rollWait&&(colorSensors.getColorLeft()==0||colorSensors.getColorLeft()==0)){
            flywheel.update(launchVel);
            telemetry.addData("launchVel", flywheel.getVelocity());
            telemetry.update();
            follower.update();
        }
    }
    public void chooseI(int targetColor, int leftColor, int rightColor){
        if(leftColor==targetColor&&rightColor==targetColor){
            if(distanceSensors.getSide()==1){
                kickRight();
            }else {
                kickLeft();
            }
        } else if (leftColor==targetColor) {
            kickLeft();
        } else if (rightColor==targetColor) {
            kickRight();
        } else{
            if(distanceSensors.getSide()==1){
                kickRight();
            }
            else {
                kickLeft();
            }
        }
    }
    public void kickLeft(){
        while((Math.abs(flywheel.getVelocity()-launchVel)>launchTol)||ballKickers.getLeftPos()<DownLeftPos||ballKickers.getRightPos()>DownRightPos){
            flywheel.update(launchVel);
            follower.update();
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
        while((Math.abs(flywheel.getVelocity()-launchVel)>launchTol)||ballKickers.getLeftPos()<DownLeftPos||ballKickers.getRightPos()>DownRightPos){
            follower.update();
            flywheel.update(launchVel);
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
        while((Math.abs(flywheel.getVelocity()-launchVel)>launchTol)||ballKickers.getLeftPos()<DownLeftPos||ballKickers.getRightPos()>DownRightPos){
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