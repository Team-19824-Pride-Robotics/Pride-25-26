package org.firstinspires.ftc.teamcode.V1;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.pathConstraints;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.V1.subsystems.intake;
import org.firstinspires.ftc.teamcode.V1.subsystems.flywheel;
import org.firstinspires.ftc.teamcode.V1.subsystems.ballKickers;
import org.firstinspires.ftc.teamcode.V1.subsystems.limelight;
import org.firstinspires.ftc.teamcode.V1.subsystems.colorSensors;
import org.firstinspires.ftc.teamcode.V1.subsystems.distanceSensors;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
@Configurable
public class Teleop extends LinearOpMode {
    //Pedro stuff
    private Follower follower;
    public Pose currentPose;
    public Pose startingPose = new Pose(28, 130, Math.toRadians(136));
    private TelemetryManager telemetryM;

    //mech subsystem declarations
    private intake intake;
    private flywheel flywheel;
    private ballKickers ballKickers;
    private limelight limelight;
    private colorSensors colorSensors;
    private distanceSensors distanceSensors;
    //fun variables
    //Flywheel Velocities
    private static double defaultLaunchVel =1050;

    //Kicker positions
    private static double UpRightPos=260;
    private static double UpLeftPos=210;
    private double DownRightPos=309;
    private double DownLeftPos=114;

    //Alliance color logic
    private boolean allianceSelected=false; //only allows teleop to progress after alliance is selected
    private boolean redAlliance=false; // durrrrr

    //Launching logic
    private boolean launchLeft=false; //When true left chamber launches and automatically retracts
    private boolean launchRight=false; //When true right chamber launches and automatically retracts
    private boolean launchE = false; //kickstarts efficient launch sequence
    private boolean indexMode = false; //When false, robot launches artifacts as efficiently as possible, when true, launching is manual to index
    private boolean block = true; //Kicks left kicker halfway to block space and prevent robot from picking up 4
    private boolean stopIntaking=false; //Stops intake once robot is full, resets after trigger is released
    private double launchVel=0; //desired launch velocity of launcher
    private int launchQueue = 0; //artifact queue, used in E mode

    //Park positions
    private static double parkX=0; //park pos x
    private static double parkY=0; //park pos y

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
        intake = new intake(hardwareMap );
        limelight = new limelight(hardwareMap);
        flywheel = new flywheel(hardwareMap);
        ballKickers = new ballKickers(hardwareMap);
        colorSensors = new colorSensors(hardwareMap);
        distanceSensors= new distanceSensors(hardwareMap);

        //init mechs
        limelight.init();
        limelight.setPipeline(3);
        flywheel.init();
        ballKickers.retractLeft();
        ballKickers.retractRight();
        intake.setPower(0);

        //init pedro
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startingPose);
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

        follower.startTeleopDrive();
        while (opModeIsActive()) {


            //localization

            follower.update();
            telemetryM.update();
            if(limelight.isValid()) {
                follower.setPose(limelight.relocalize(follower.getHeading()));
            }
//drive control
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,   // forward/back
                    -gamepad1.left_stick_x,   // strafe
                    -gamepad1.right_stick_x,  // rotation
                    true                     // field-centric=false
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
                if (gamepad1.a) { //Auto-park

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

            //intake and block logic

            //Check if transfer is full before unblocking
            if(block) {
                if (distanceSensors.getCount() > 1) {
                    block = false;
                    stopIntaking=true;
                }
            }
            if((gamepad1.right_trigger>0.1 || gamepad1.left_trigger>0.1) && !stopIntaking) {
                intake.setPower(gamepad1.left_trigger - gamepad1.right_trigger); //Sets power based on triggers
            }
            if(gamepad1.left_trigger<0.1){
                stopIntaking=false; //resets intake
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

                    if (limelight.getDistance() == -1) {
                        launchVel = defaultLaunchVel;
                    } else {
                        launchVel = lut.get(limelight.getDistance());
                    }



                //Kicker logic for non index mode
                if (!indexMode) {
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

                    if (launchRight || launchLeft){
                            block=false;
                    }

                    if(block){
                        ballKickers.block();
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
                                block=true;
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
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading", follower.getPose().getHeading());
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
