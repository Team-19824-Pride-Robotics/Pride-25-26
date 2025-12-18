package org.firstinspires.ftc.teamcode.V2;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "blueAuto")
@Configurable

public class blueAuto extends OpMode {





    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(28, 130, Math.toRadians(136)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(55, 100, Math.toRadians(140)); // Scoring Pose of our robot. It is facing the goal at a 136 degree angle.
    private final Pose lineup1Pose = new Pose(55, 87, Math.toRadians(180)); // Highest (First Set)
    private final Pose gobble1Pose = new Pose(18, 87, Math.toRadians(180)); // Highest (First Set)
    private final Pose lineup2Pose = new Pose(55, 64, Math.toRadians(180)); // Middle (Second Set)
    private final Pose gobble2Pose = new Pose(9, 64, Math.toRadians(180)); // Middle (Second Set)
    private final Pose scorePose2 = new Pose(55, 100, Math.toRadians(140));
    private final Pose scorePose3 = new Pose(55, 100, Math.toRadians(140));
    private final Pose lineup3Pose = new Pose(55, 43, Math.toRadians(180)); // Middle (Second Set)
    private final Pose gobble3Pose = new Pose(12, 43, Math.toRadians(180)); // Middle (Second Set)
    private PathChain scorePreload, grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;


    public void buildPaths() {

        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setConstantHeadingInterpolation(startPose.getHeading())
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
                .addPath(new BezierLine(gobble1Pose, scorePose))
                .setLinearHeadingInterpolation(gobble1Pose.getHeading(), scorePose2.getHeading())
                .build();

        /* grabPickup2 PathChain --> lines up for the second set of artifacts, then
           turns on the intake and gobbles them up in a line  */

        grabPickup2 = follower.pathBuilder()

                .addPath(new BezierLine(scorePose, lineup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), lineup2Pose.getHeading())
                .addPath(new BezierLine(lineup2Pose, gobble2Pose))
                .setConstantHeadingInterpolation(lineup2Pose.getHeading())
                .build();

        /* scorePickup2 PathChain --> moves from the gobble2Pose back to the scoring position  */

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(lineup2Pose, scorePose))
                .setLinearHeadingInterpolation(lineup2Pose.getHeading(), scorePose3.getHeading())
                .build();



        grabPickup3 = follower.pathBuilder()

                .addPath(new BezierLine(scorePose, lineup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), lineup3Pose.getHeading())
                .addPath(new BezierLine(lineup3Pose, gobble3Pose))
                .setConstantHeadingInterpolation(lineup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(lineup3Pose, scorePose))
                .setLinearHeadingInterpolation(lineup3Pose.getHeading(), scorePose3.getHeading())
                .build();

    }



    public void autonomousPathUpdate() {
        switch (pathState) {

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

//set the speed and back up
            case 0:
                follower.setMaxPower(1);  //slow down the path following if necessary
                follower.followPath(scorePreload);
                setPathState(1);
                break;
//Launches artifacts, then sets the transfer and intake on and sets the robot to slow. Then it will run grab pickup 1
            case 1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {

                    /* score the preload artifacts by spinning the transfer */




                    follower.followPath(grabPickup1,true);

                    setPathState(2);
                }
                break;
//picks up the balls, makes the robot fast again, and then gets into position to score
            case 2:

                if(!follower.isBusy()) {

                    follower.followPath(scorePickup1,true);
                    setPathState(3);
                }
                break;
//launches the balls, then sets the intake and transfer on, closes the servo and slows it down then it will pick up the balls
            case 3:

                if(!follower.isBusy()) {


                    follower.followPath(grabPickup2,true);
                    setPathState(4);
                }
                break;
//gets into scoring position
            case 4:

                if(!follower.isBusy()) {


                    follower.followPath(scorePickup2,true);

                    setPathState(5);
                }
                break;
//scores the balls after opening the servo and gets back in position to pick up the balls
            case 5:


                if(!follower.isBusy()) {

                    follower.followPath(grabPickup3,true);
                    setPathState(6);
                }
                break;

            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */



                    follower.followPath(scorePickup3,true);


                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */

                    setPathState(7);
                }
                break;

            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {



                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
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





    public Runnable intake_change(double power) {

        return null;
    }






}