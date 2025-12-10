package org.firstinspires.ftc.teamcode.V2;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;


import org.firstinspires.ftc.teamcode.V2.subsystems.Intake;
import org.firstinspires.ftc.teamcode.V2.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.V2.subsystems.BallKickers;
import org.firstinspires.ftc.teamcode.V2.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.V2.subsystems.ColorSensors;
import org.firstinspires.ftc.teamcode.V2.subsystems.DistanceSensors;
@TeleOp
@Configurable
public class Teleop extends LinearOpMode {
    //mech subsystem declarations
    private Intake intake;
    private Flywheel flywheel;
    private BallKickers ballKickers;
    private Limelight limelight;

    private ColorSensors colorSensors;
    private DistanceSensors distanceSensors;
    //fun variables
    private static double ejectVel = 600;
    private static double defaultLaunchVel =1000;
    private static double spinUpPower = 1;
    private static double UpRightPos=185;
    private static double UpLeftPos=240;
    private boolean launchLeft=false;
    private boolean launchRight=false;
    private boolean launchE = false; //kickstarts efficient launch sequence
    private boolean indexMode = false;
    private boolean eject = false;
    private double launchVel=0;
    private double DownRightPos=309;
    private double DownLeftPos=114;
    private int launchQueue = 0;

    //Other stuff
    InterpLUT lut = new InterpLUT();



    @Override
    public void runOpMode() throws InterruptedException {


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

        //init mechs
        limelight.init();
        limelight.setPipeline(3);
        flywheel.initialize();
        ballKickers.initialize();
        intake.initialize();



        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            //drive control



            if(gamepad2.dpad_left){
                limelight.setPipeline(3);
            }
            if(gamepad2.dpad_right){
                limelight.setPipeline(4);
            }


            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;


            if(!gamepad1.a) {
                frontLeftMotor.setPower(frontLeftPower * (1 - gamepad1.right_trigger) * 0.7);
                backLeftMotor.setPower(backLeftPower * (1 - gamepad1.right_trigger) * 0.7);
                frontRightMotor.setPower(frontRightPower * (1 - gamepad1.right_trigger) * 0.7);
                backRightMotor.setPower(backRightPower * (1 - gamepad1.right_trigger) * 0.7);
            }
            //////////////
            //Mechansims//
            //////////////

            //intake
            intake.setPower(gamepad2.right_trigger-gamepad2.left_trigger);

            //indexing selecting
            if(gamepad2.start){
                indexMode=true;
            }
            if(gamepad2.back){
                indexMode=false;
            }



            //ball kicking kickoff
            if(indexMode) {
                if (gamepad2.right_bumper) {
                    launchRight = true;
                }
                if (gamepad2.left_bumper) {
                    launchLeft = true;
                }
                if(gamepad2.b){
                    launchLeft = false;
                    launchRight = false;
                }
                launchE=false;
            }
            else{
                if(gamepad2.right_bumper || gamepad2.left_bumper){
                    launchE=true;
                    launchQueue=3;
                }
                if(gamepad2.b){
                    launchE=false;
                    launchQueue=0;
                }
            }

            //launch logic

            if(limelight.getDistance()==-1){
                launchVel=defaultLaunchVel;
            }else{
                launchVel=lut.get(limelight.getDistance());
            }



            //Kicker logic for non index mode
            if(!indexMode) {
                //Kickstart first launch
                if(launchE){
                    if(distanceSensors.getSide()==1){
                        launchRight=true;
                        launchLeft=false;
                    } else{
                        launchRight=false;
                        launchLeft=true;
                    }
                    launchE=false;
                }

                //Launch logic
                if (launchRight) {
                    ballKickers.retractLeft();
                    if ((flywheel.getVelocity() < launchVel + 40 && flywheel.getVelocity() > launchVel - 40) && ballKickers.getRightPos() < DownRightPos) {
                        if (launchQueue == 1) {
                            if (colorSensors.getColorRight() != -1) {
                                ballKickers.kickRight();
                            }
                        } else {
                            ballKickers.kickRight();
                        }
                    }
                }
                if (launchLeft) {
                    ballKickers.retractRight();
                    if ((flywheel.getVelocity() < launchVel + 40 && flywheel.getVelocity() > launchVel - 40) && ballKickers.getLeftPos() < DownLeftPos) {
                        if (launchQueue == 1) {
                            if (colorSensors.getColorLeft() != -1) {
                                ballKickers.kickLeft();
                            }
                        } else {
                            ballKickers.kickLeft();
                        }
                    }
                }

                //Retraction logic
                if (launchLeft && ballKickers.getLeftPos() < UpLeftPos) {
                    ballKickers.retractLeft();
                    launchLeft = false;
                    if(launchQueue>0){
                        launchRight=true;
                        launchQueue--;
                    }
                }
                if (launchRight && ballKickers.getRightPos() > UpRightPos) {
                    ballKickers.retractRight();
                    launchRight = false;
                    if(launchQueue>0){
                        launchLeft=true;
                        launchQueue--;
                    }
                }
            } else{
                if(launchRight) {
                    ballKickers.retractLeft();
                    if (flywheel.getVelocity() < launchVel+40 && flywheel.getVelocity() > launchVel-40) {
                        ballKickers.kickRight();
                    }
                }
                if(launchRight&&ballKickers.getRightPos()>UpRightPos){
                    ballKickers.retractRight();
                    launchRight=false;
                }

                if(launchLeft) {
                    ballKickers.retractRight();
                    if (flywheel.getVelocity() < launchVel+40 && flywheel.getVelocity() > launchVel-40) {
                        ballKickers.kickLeft();
                    }
                }
                if(launchLeft&&ballKickers.getLeftPos()<UpLeftPos){
                    ballKickers.retractLeft();
                    launchLeft=false;
                }
            }
            //update mechs
            flywheel.update(launchVel);
            ballKickers.update();
            intake.update();
            transferChanneler.update();

            telemetry.addData("Angle From Goal", limelight.getAngle());
            telemetry.addData("Wheel speed ", flywheel.getVelocity());
            telemetry.addData("Desired wheel speed", launchVel);
            telemetry.addData("Distance From Goal: ", limelight.getDistance());
            telemetry.addData("Right kciker pos: ", ballKickers.getRightPos());
            telemetry.addData("Desired pos: ", ballKickers.getRightDesiredPos());
            telemetry.update();

        }
    }
}
//green bull party at 4470 lennox blvd. november 3rd 2030
