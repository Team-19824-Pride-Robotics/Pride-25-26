package org.firstinspires.ftc.teamcode.V1;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.pathConstraints;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.V1.subsystems.ballKickers;
import org.firstinspires.ftc.teamcode.V1.subsystems.colorSensors;
import org.firstinspires.ftc.teamcode.V1.subsystems.distanceSensors;
import org.firstinspires.ftc.teamcode.V1.subsystems.flywheel;
import org.firstinspires.ftc.teamcode.V1.subsystems.intake;
import org.firstinspires.ftc.teamcode.V1.subsystems.limelight;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
@Configurable

public class IntermediateTeleop extends LinearOpMode {

    //pedro stuff
    private Follower follower;
    public Pose currentPose;
    public static Pose startingPose = new Pose();
    //mech subsystem declarations
    private intake intake;
    private flywheel flywheel;
   private ballKickers ballKickers;
    private limelight limelight;
    private colorSensors colorSensors;
    private distanceSensors distanceSensors;
    //intake logic
    private static double intakePowerDampening = 0.8;
    //launch logic
    private static double UpRightPos=135;
    private static double UpLeftPos=220;
    private static double DownRightPos=90;
    private static double DownLeftPos=290;
    private boolean launchLeft=false;
    private boolean launchRight=false;
    private boolean indexMode=false;
    private boolean launchE=false;
    private int launchQueue=0;
    //flywheel logic\
    private static double defaultLaunchVel=1100;
    private double launchVel=defaultLaunchVel;
    private boolean disableFlywheel=false;
    private boolean unJam=false;

    //Alliance selection
    private boolean allianceSelected=false;
    private boolean redAlliance=false;
    //Pedro pathing party vars
    boolean headingLock = false;
    private boolean automatedDrive=false;
    boolean switchDrive=false;
    private static double scoreHeadingTolerance=0.1;
    private static double scoreTranslationalConstraint=0.5;
    //Hardware reads
    private double leftKickerPos;
    private double rightKickerPos;
    private double flywheelVelocity;



    //Other stuff
    InterpLUT lut = new InterpLUT();



    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startingPose);
        PIDFController controller = new PIDFController(follower.constants.coefficientsHeadingPIDF);
        //Intep table setup
        lut.add(0, 1000);
        lut.add(48, 1080);
        lut.add(55, 1100);
        lut.add(60, 1100);
        lut.add(65, 1120);
        lut.add(70, 1120);
        lut.add(75, 1140);
        lut.add(80, 1140);
        lut.add(85, 1200);
        lut.add(90, 1240);
        lut.add(95, 1280);
        lut.add(100, 1300);
        lut.add(105, 1320);
        lut.add(110, 1360);
        lut.add(115, 1380);
        lut.add(120, 1400);
        lut.add(125, 1420);
        lut.add(130, 1440);

        lut.createLUT();

        // Declare motor ok
        // Absolutely yes make ID's match configuration
        IMU imu = hardwareMap.get(IMU.class, "imu");
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("fLD");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("bLD");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("fRD");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("bRD");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        //reverse drive motors
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Declare mechs
        intake = new intake(hardwareMap);
        limelight = new limelight(hardwareMap);
        flywheel = new flywheel(hardwareMap);
        ballKickers = new ballKickers(hardwareMap);
        colorSensors = new colorSensors(hardwareMap);
        distanceSensors = new distanceSensors(hardwareMap);

        //init mechs
        limelight.init();
        limelight.setPipeline(3);
        flywheel.init();
        ballKickers.retractRight();
        ballKickers.retractLeft();

        if(startingPose == null){
            startingPose = new Pose(144 - 88, 9, Math.toRadians(90));
        }
        //alliance selection
        while(!allianceSelected){
            telemetry.addData("Select alliance with Dpad","");
            telemetry.addData("Left=Blue","");
            telemetry.addData("Right=Red","");
            if(gamepad1.dpad_right){
                redAlliance=true;
                allianceSelected=true;
                limelight.setPipeline(4);
            } if(gamepad1.dpad_left){
                redAlliance=false;
                allianceSelected=true;
                limelight.setPipeline(3);
            }
            telemetry.update();
        }



        waitForStart();

        if (isStopRequested()) return;
        follower.startTeleopDrive();
        while (opModeIsActive()) {
            follower.update();
            //Hardware reads
            leftKickerPos= ballKickers.getLeftPos();
            rightKickerPos= ballKickers.getRightPos();
            currentPose=follower.getPose();
            flywheelVelocity=flywheel.getVelocity();


            //Alliance reselection selection
            if(gamepad2.dpad_left){
                redAlliance=false;
                limelight.setPipeline(3);
            }
            if(gamepad2.dpad_right){
                redAlliance=true;
                limelight.setPipeline(4);
            }
            //heading lock and drive control
            if(headingLock) {
                controller.setCoefficients(follower.constants.coefficientsHeadingPIDF);
                double targetX;
                if (redAlliance) {
                    targetX = 144;
                } else {
                    targetX = 0;
                }
                double targetY = 144;

                double robotX = currentPose.getX();
                double robotY = currentPose.getY();

                double dx = targetX - robotX;
                double dy = targetY - robotY;

                double headingGoal = Math.atan(dy / dx);

                if (follower.getCurrentPath() == null) {
                    controller.updateError(0);
                }
                controller.updateError(MathFunctions.getTurnDirection(currentPose.getHeading(), headingGoal) * MathFunctions.getSmallestAngleDifference(currentPose.getHeading(), headingGoal));
            }

            if (gamepad1.a) { //Go to far zone launch
                automatedDrive = true;
                PathChain pathChain;// start
// end
                if(redAlliance) {
                    pathChain = follower.pathBuilder()
                            .addPath(
                                    new Path(
                                            new BezierLine(
                                                    new Pose(follower.getPose().getX(), follower.getPose().getY()), // start
                                                    new Pose(88, 19, Math.toRadians(79))                                        // end
                                            ),
                                            pathConstraints
                                    )
                            )
                            .setLinearHeadingInterpolation(follower.getHeading(), Math.toRadians(79))
                            .setHeadingConstraint(Math.toRadians(scoreHeadingTolerance))
                            .setTranslationalConstraint(scoreTranslationalConstraint)
                            .build();
                } else{
                    pathChain = follower.pathBuilder()
                            .addPath(
                                    new Path(
                                            new BezierLine(
                                                    new Pose(follower.getPose().getX(), follower.getPose().getY()), // start
                                                    new Pose(144 - 88, 19, Math.toRadians(111))                                         // end
                                            ),
                                            pathConstraints
                                    )
                            )
                            .setHeadingConstraint(Math.toRadians(scoreHeadingTolerance))
                            .setTranslationalConstraint(scoreTranslationalConstraint)
                            .setLinearHeadingInterpolation(follower.getHeading(), Math.toRadians(111))
                            .build();
                }
                follower.followPath(pathChain);

            }
            if (headingLock) {
                follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, controller.run());
            }
            else{
                follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            }

            if(gamepad1.b){
                headingLock=false;
                automatedDrive=false;
                follower.startTeleopDrive();
            } if(gamepad1.a){
                headingLock=true;
            }
            if(gamepad1.x){
                if(limelight.isValid()){
                    follower.setPose(limelight.relocalize(follower.getHeading()));
                }
            }
            //////////////
            //Mechansims//
            //////////////

            //intake
                intake.setPower((gamepad1.right_trigger-gamepad1.left_trigger)*intakePowerDampening);
            //Emergency flywheel control
            if(gamepad1.dpad_down){
                unJam=true;
                disableFlywheel=false;
            }
            if(gamepad1.dpad_left){
                unJam=false;
                disableFlywheel=true;
            }
            if(gamepad1.dpad_up){
                unJam=false;
                disableFlywheel=false;
            }


            //Launch type selection
            if(gamepad1.start){
                indexMode=true;
            } if(gamepad1.share){
                indexMode=false;
            }
            if (indexMode) {
                if (gamepad1.right_bumper) {
                    launchRight = true;
                }
                if (gamepad1.left_bumper) {
                    launchLeft = true;
                }
                if (gamepad1.b) {
                    launchLeft = false;
                    launchRight = false;
                }
                launchE = false;
            } else {
                if (gamepad1.right_bumper || gamepad1.left_bumper) {
                    launchE = true;
                    launchQueue = 3;
                }
                if (gamepad1.b) {
                    launchE = false;
                    launchQueue = 0;
                }
            }

            //launch logic
            if(!unJam&&!disableFlywheel) {
                if (limelight.getDistance() != -1) {
                    launchVel = lut.get(limelight.getDistance());
                }
            } else if (disableFlywheel) {
                launchVel=0;
            }


            //Kicker logic for non index mode
            if (!indexMode) {
                //Kickstart first launch
                if (launchE) {
                    if (!launchLeft && !launchRight) {
                        if(launchQueue>1) {
                            if (distanceSensors.getSide() == 1) {
                                launchRight = true;
                            } else {
                                launchLeft = true;
                            }
                            launchQueue--;
                        }
                        else{
                            launchRight=true;
                            launchLeft=true;
                            launchQueue--;
                            launchE=false;
                        }
                    }

                }



                //Launch logic
                if (launchRight) {
                    ballKickers.retractLeft();
                    if (Math.abs(flywheel.getVelocity()-launchVel)<20 && rightKickerPos < DownRightPos) {
                        if (launchQueue == 1) {
                            if(colorSensors.getColorLeft()>0||colorSensors.getColorRight()>0){
                                ballKickers.kickRight();
                                ballKickers.kickLeft();
                            }
                        } else {
                            ballKickers.kickRight();
                        }
                    }
                }
                if (launchLeft) {
                    ballKickers.retractRight();
                    if (Math.abs(flywheel.getVelocity()-launchVel)<20  && leftKickerPos < DownLeftPos) {
                        if (launchQueue == 1) {
                            if(colorSensors.getColorLeft()>0||colorSensors.getColorRight()>0) {
                                ballKickers.kickRight();
                                ballKickers.kickLeft();
                            }
                        } else {
                            ballKickers.kickLeft();
                        }
                    }
                }

                //Retraction logic
                if (launchLeft && leftKickerPos < UpLeftPos) {
                    ballKickers.retractLeft();
                    ballKickers.retractRight();
                    launchLeft = false;
                    if (launchQueue > 1) {
                        launchRight = true;
                        launchQueue--;
                    }
                }
                if (launchRight && rightKickerPos > UpRightPos) {
                    ballKickers.retractRight();
                    ballKickers.retractLeft();
                    launchRight = false;
                    if (launchQueue > 1) {
                        launchLeft = true;
                        launchQueue--;
                    }
                }
            } else {
                if (launchRight) {
                    ballKickers.retractLeft();
                    if (Math.abs(flywheel.getVelocity()-launchVel)<20  && rightKickerPos < DownRightPos) {
                        ballKickers.kickRight();
                    }
                }
                if (launchRight && rightKickerPos > UpRightPos) {
                    ballKickers.retractRight();
                    launchRight = false;
                }

                if (launchLeft) {
                    ballKickers.retractRight();
                    if (Math.abs(flywheel.getVelocity()-launchVel)<20  && leftKickerPos < DownLeftPos) {
                        ballKickers.kickLeft();
                    }
                }
                if (launchLeft && leftKickerPos < UpLeftPos) {
                    ballKickers.retractLeft();
                    launchLeft = false;
                }
            }


            //update mechs
            if(!unJam) {
                flywheel.update(launchVel);
            } else{
                flywheel.setPower(-1);
            }
            ballKickers.update();
            intake.update();

            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading", follower.getPose().getHeading());
            telemetry.addData("Angle From Goal", limelight.getAngle());
            telemetry.addData("Wheel speed ", flywheelVelocity);
            telemetry.addData("Desired wheel speed", launchVel);
            telemetry.addData("Distance From Goal: ", limelight.getDistance());
            telemetry.addData("Right kciker pos: ", rightKickerPos);
            telemetry.addData("Left kciker pos: ", leftKickerPos);
            telemetry.addData("launchE", launchE);
            telemetry.addData("launchQueue", launchQueue);
            telemetry.addData("launchRight", launchRight);
            telemetry.addData("launchQueue", launchLeft);
            telemetry.update();

        }
    }

}

//green bull party at 4470 lennox blvd. november 3rd 2030
