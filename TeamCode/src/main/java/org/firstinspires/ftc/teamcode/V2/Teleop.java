package org.firstinspires.ftc.teamcode.V2;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.pathConstraints;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.V2.subsystems.Intake;
import org.firstinspires.ftc.teamcode.V2.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.V2.subsystems.BallKickers;
import org.firstinspires.ftc.teamcode.V2.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.V2.subsystems.ColorSensors;
import org.firstinspires.ftc.teamcode.V2.subsystems.DistanceSensors;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
@Configurable
public class Teleop extends LinearOpMode {
    //Pedro stuff
    private Follower follower;
    public static Pose startingPose; // optional starting pose
    private TelemetryManager telemetryM;

    //mech subsystem declarations
    private Intake intake;
    private Flywheel flywheel;
    private BallKickers ballKickers;
    private Limelight limelight;

    private ColorSensors colorSensors;
    private DistanceSensors distanceSensors;
    //fun variables
    //Flywheel Velocities
    private static double ejectVel = 600;
    private static double defaultLaunchVel =1000;

    //Kicker positions
    private static double UpRightPos=185;
    private static double UpLeftPos=240;
    private double DownRightPos=309;
    private double DownLeftPos=114;

    //Alliance color logic
    private boolean allianceSelected=false;
    private boolean redAlliance=false;

    //Launching logic
    private boolean launchLeft=false;
    private boolean launchRight=false;
    private boolean launchE = false; //kickstarts efficient launch sequence
    private boolean indexMode = false;
    private boolean eject = false;
    private boolean intaking=false;
    private double launchVel=0;
    private int launchQueue = 0;

    //Park positions
    private static double parkX=0;
    private static double parkY=0;

    //Other stuff
    InterpLUT lut = new InterpLUT();
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {


        //Interp table setup
        lut.add(45, 950);
        lut.add(48, 1000);
        lut.add(56, 1100);
        lut.add(60, 1120);
        lut.add(70, 1140);
        lut.add(80, 1160);
        lut.add(90, 1180);
        lut.add(100, 1200);
        lut.add(110, 1260);
        lut.add(120, 1280);
        lut.add(130, 1340);
        lut.add(140, 1360);

        lut.createLUT();


        //Declare mechs
        intake = new Intake();
        limelight = new Limelight(hardwareMap);
        flywheel = new Flywheel();
        ballKickers = new BallKickers();
        colorSensors = new ColorSensors(hardwareMap);
        distanceSensors= new DistanceSensors(hardwareMap);

        //init mechs
        limelight.init();
        limelight.setPipeline(3);
        flywheel.initialize();
        ballKickers.initialize();
        intake.initialize();

        //init pedro
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        while(!allianceSelected){
            telemetry.addData("Select alliance with Dpad","");
            telemetry.addData("Left=Blue","");
            telemetry.addData("Right=Red","");
            if(gamepad1.dpad_right){
                redAlliance=true;
                allianceSelected=true;
                limelight.setPipeline(3);
            } if(gamepad1.dpad_left){
                redAlliance=false;
                allianceSelected=true;
                limelight.setPipeline(4);
            }
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {


            //drive control
            follower.update();
            telemetryM.update();

            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,   // forward/back
                    -gamepad1.left_stick_x,   // strafe
                    -gamepad1.right_stick_x,  // rotation
                    false                     // field-centric
            );
            //Aim at goal
            if (gamepad1.y) {
                double targetX = redAlliance ? 144 : 0;
                double targetY = 144;

                double robotX = follower.getPose().getX();
                double robotY = follower.getPose().getY();

                double dx = targetX - robotX;
                double dy = targetY - robotY;

                double headingToTarget = Math.atan2(dy, dx);

                PathChain turnChain = follower.pathBuilder()
                        .setLinearHeadingInterpolation(follower.getHeading(), headingToTarget)
                        .build();

                follower.followPath(turnChain);

            }
                if (gamepad1.a) { // Press A to auto-move
                    Pose targetPose = new Pose(100, 50, Math.toRadians(90));

                    PathChain pathChain = follower.pathBuilder()
                            .addPath(
                                    new Path(
                                            new BezierLine(
                                                    new Pose(follower.getPose().getX(), follower.getPose().getY()), // start
                                                    new Pose(parkX, parkY)                                         // end
                                            ),
                                            pathConstraints
                                    )
                            )
                            .setLinearHeadingInterpolation(follower.getHeading(), Math.PI)
                            .build();

                    follower.followPath(pathChain);
                }






            //////////////
            //Mechansims//
            //////////////

            //intake
            if(gamepad2.right_trigger>0.1 || gamepad2.left_trigger>0.1) {
                intake.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
                intaking=true;
            }
            if(intaking && gamepad2.right_trigger<0.1 && gamepad2.left_trigger<0.1){
                if(distanceSensors.getCount()>1){
                    eject=true;
                }
                intaking=false;
            }
            //indexing selecting
            if(gamepad2.start){
                indexMode=true;
            }
            if(gamepad2.back){
                indexMode=false;
            }



            //ball kicking kickoff

                if (indexMode) {
                    if (gamepad2.right_bumper) {
                        launchRight = true;
                    }
                    if (gamepad2.left_bumper) {
                        launchLeft = true;
                    }
                    if (gamepad2.b) {
                        launchLeft = false;
                        launchRight = false;
                    }
                    launchE = false;
                } else if (eject) {
                    launchLeft=true;
                    eject=false;
                } else {
                    if (gamepad2.right_bumper || gamepad2.left_bumper) {
                        launchE = true;
                        launchQueue = 3;
                    }
                    if (gamepad2.b) {
                        launchE = false;
                        launchQueue = 0;
                    }
                }

                //launch logic
                if(!eject) {
                    if (limelight.getDistance() == -1) {
                        launchVel = defaultLaunchVel;
                    } else {
                        launchVel = lut.get(limelight.getDistance());
                    }
                } else{
                    launchVel=ejectVel;
                }

                //Kicker logic for non index mode
                if (!indexMode || !eject) {
                    //Kickstart first launch
                    if (launchE) {
                        if (distanceSensors.getSide() == 1) {
                            launchRight = true;
                            launchLeft = false;
                        } else {
                            launchRight = false;
                            launchLeft = true;
                        }
                        launchE = false;
                    }

                    //Launch logic
                    if (launchRight) {
                        ballKickers.retractLeft();
                        if ((flywheel.getVelocity() < launchVel + 40 && flywheel.getVelocity() > launchVel - 40) && ballKickers.getRightPos() < DownRightPos) {
                            if (launchQueue == 1) {
                                ballKickers.kickRight();
                                ballKickers.kickLeft();
                            } else {
                                ballKickers.kickRight();
                            }
                        }
                    }
                    if (launchLeft) {
                        ballKickers.retractRight();
                        if ((flywheel.getVelocity() < launchVel + 40 && flywheel.getVelocity() > launchVel - 40) && ballKickers.getLeftPos() < DownLeftPos) {
                            if (launchQueue == 1) {
                                ballKickers.kickRight();
                                ballKickers.kickLeft();
                            } else {
                                ballKickers.kickLeft();
                            }
                        }
                    }

                    //Retraction logic
                    if (launchLeft && ballKickers.getLeftPos() < UpLeftPos) {
                        ballKickers.retractLeft();
                        launchLeft = false;
                        if (launchQueue > 0) {
                            launchRight = true;
                            launchQueue--;
                        }
                    }
                    if (launchRight && ballKickers.getRightPos() > UpRightPos) {
                        ballKickers.retractRight();
                        launchRight = false;
                        if (launchQueue > 0) {
                            launchLeft = true;
                            launchQueue--;
                        }
                    }
                } else {
                    if (launchRight) {
                        ballKickers.retractLeft();
                        if (flywheel.getVelocity() < launchVel + 40 && flywheel.getVelocity() > launchVel - 40  && ballKickers.getRightPos() < DownRightPos) {
                            ballKickers.kickRight();
                        }
                    }
                    if (launchRight && ballKickers.getRightPos() > UpRightPos) {
                        ballKickers.retractRight();
                        launchRight = false;
                    }

                    if (launchLeft) {
                        ballKickers.retractRight();
                        if (flywheel.getVelocity() < launchVel + 40 && flywheel.getVelocity() > launchVel - 40  && ballKickers.getLeftPos() < DownLeftPos) {
                            ballKickers.kickLeft();
                        }
                    }
                    if (launchLeft && ballKickers.getLeftPos() < UpLeftPos) {
                        ballKickers.retractLeft();
                        launchLeft = false;
                    }
                }

            //update mechs
            flywheel.update(launchVel);
            intake.update();

            //Basic telemetry
            telemetry.addData("Angle From Goal", limelight.getAngle());
            telemetry.addData("Wheel speed ", flywheel.getVelocity());
            telemetry.addData("Desired wheel speed", launchVel);
            telemetry.addData("Distance From Goal: ", limelight.getDistance());
            telemetry.addData("Right kicker pos: ", ballKickers.getRightPos());
            telemetry.addData("Left kicker pos: ", ballKickers.getLeftPos());
            telemetry.addData("Extra Count", distanceSensors.getCount());
            telemetry.addData("Side", distanceSensors.getSide());
            telemetry.addData("runtime", runtime.seconds());
            telemetry.update();

            //Panels telemetry
            // Debug telemetry
            telemetryM.debug("position", follower.getPose());
            telemetryM.debug("velocity", follower.getVelocity());

        }
    }
}
//green bull party at 4470 lennox blvd. november 3rd 2030
