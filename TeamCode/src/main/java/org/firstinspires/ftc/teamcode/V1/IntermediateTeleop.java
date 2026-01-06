package org.firstinspires.ftc.teamcode.V1;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
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
    public Pose startingPose = new Pose(64, 140, Math.toRadians(90));
//mech subsystem declarations
    private intake intake;
    private flywheel flywheel;
   private ballKickers ballKickers;
    private limelight limelight;
    private colorSensors colorSensors;
    private distanceSensors distanceSensors;
    //fun variables
    private int[][] balls;
    private static double ejectVel = 600;
    private static double defaultLaunchVel =1100;
    private static double spinUpPower = 1;
    private static double UpRightPos=260;
    private static double UpLeftPos=220;
    private boolean launchLeft=false;
    private boolean launchRight=false;
    private double launchVel=0;
    private static double DownRightPos=210;
    private static double DownLeftPos=280;
    private boolean automatedDrive=false;
    private boolean allianceSelected=false;
    private boolean redAlliance=false;
    private double leftKickerPos;
    private double rightKickerPos;
    private boolean indexMode=false;
    private boolean launchE=false;
    private int launchQueue=3;
    private

    //Other stuff
    InterpLUT lut = new InterpLUT();



    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startingPose);

        //Intep table setup
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
            follower.update();
            leftKickerPos= ballKickers.getLeftPos();
            rightKickerPos= ballKickers.getRightPos();

            //drive control


            if (gamepad1.options) {
                imu.resetYaw();
            }
            if(gamepad2.dpad_left){
                limelight.setPipeline(3);
            }
            if(gamepad2.dpad_right){
                limelight.setPipeline(4);
            }


            if(!automatedDrive) {
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y,   // forward/back
                        -gamepad1.left_stick_x,   // strafe
                        -gamepad1.right_stick_x,  // rotation
                        true                     // field-centric=false
                );
            }
            //Aim at goal
            if (gamepad1.y) {
                automatedDrive = true;
                double targetX;
                if(redAlliance){
                    targetX=144;
                }else{
                    targetX=0;
                }
                double targetY = 144;

                double robotX = follower.getPose().getX();
                double robotY = follower.getPose().getY();

                double dx = targetX - robotX;
                double dy = targetY - robotY;

                double headingToTarget = Math.atan(dy/dx)+Math.toRadians(180);

                Pose targetPose = new Pose(robotX, robotY, headingToTarget);
                follower.holdPoint(targetPose);


            }
            if(gamepad1.b){
                follower.startTeleopDrive();
                automatedDrive = false;
            }
            //////////////
            //Mechansims//
            //////////////

            //intake
                intake.setPower(gamepad2.right_trigger-gamepad2.left_trigger);

            //flywheel

            //power calculations



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
                    if ((flywheel.getVelocity() < launchVel + 40 && flywheel.getVelocity() > launchVel - 40) && rightKickerPos < DownRightPos) {
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
                    if ((flywheel.getVelocity() < launchVel + 40 && flywheel.getVelocity() > launchVel - 40) && leftKickerPos < DownLeftPos) {
                        if (launchQueue == 1) {
                            ballKickers.kickRight();
                            ballKickers.kickLeft();
                        } else {
                            ballKickers.kickLeft();
                        }
                    }
                }

                //Retraction logic
                if (launchLeft && leftKickerPos < UpLeftPos) {
                    ballKickers.retractLeft();
                    launchLeft = false;
                    if (launchQueue > 0) {
                        launchRight = true;
                        launchQueue--;
                    }
                }
                if (launchRight && rightKickerPos > UpRightPos) {
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
                    if (flywheel.getVelocity() < launchVel + 40 && flywheel.getVelocity() > launchVel - 40  && rightKickerPos < DownRightPos) {
                        ballKickers.kickRight();
                    }
                }
                if (launchRight && rightKickerPos > UpRightPos) {
                    ballKickers.retractRight();
                    launchRight = false;
                }

                if (launchLeft) {
                    ballKickers.retractRight();
                    if (flywheel.getVelocity() < launchVel + 40 && flywheel.getVelocity() > launchVel - 40  && leftKickerPos < DownLeftPos) {
                        ballKickers.kickLeft();
                    }
                }
                if (launchLeft && leftKickerPos < UpLeftPos) {
                    ballKickers.retractLeft();
                    launchLeft = false;
                }
            }


            //update mechs
            if(gamepad2.x){
                flywheel.setPower(-spinUpPower);
            }else {
                flywheel.update(launchVel);
            }
            ballKickers.update();
            intake.update();


            telemetry.addData("Angle From Goal", limelight.getAngle());
            telemetry.addData("Wheel speed ", flywheel.getVelocity());
            telemetry.addData("Desired wheel speed", launchVel);
            telemetry.addData("Distance From Goal: ", limelight.getDistance());
            telemetry.addData("Right kciker pos: ", ballKickers.getRightPos());
            telemetry.addData("Left kciker pos: ", ballKickers.getLeftPos());
            telemetry.addData("Desired pos: ", ballKickers.getRightDesiredPos());
            telemetry.addData("launchE", launchE);
            telemetry.addData("launchQueue", launchQueue);
            telemetry.addData("launchRight", launchRight);
            telemetry.addData("launchQueue", launchLeft);
            telemetry.update();

        }
    }
}
//green bull party at 4470 lennox blvd. november 3rd 2030
