package org.firstinspires.ftc.teamcode.prideRobotics;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;


import org.firstinspires.ftc.teamcode.prideRobotics.subsystems.intake;
import org.firstinspires.ftc.teamcode.prideRobotics.subsystems.flywheel;
import org.firstinspires.ftc.teamcode.prideRobotics.subsystems.ballKickers;
import org.firstinspires.ftc.teamcode.prideRobotics.subsystems.limelight;
import org.firstinspires.ftc.teamcode.prideRobotics.subsystems.transferChanneler;
import org.firstinspires.ftc.teamcode.prideRobotics.subsystems.colorSensors;
@TeleOp
@Configurable
public class Teleop extends LinearOpMode {
    //mech subsystem declarations
    private intake intake;
    private flywheel flywheel;
    private ballKickers ballKickers;
    private limelight limelight;
    private transferChanneler transferChanneler;
    private colorSensors colorSensors;
    //fun variables
    private int[][] balls;
    private static double ejectVel = 600;
    private static double defaultLaunchVel =1000;
    private static double UpRightPos=185;
    private static double UpLeftPos=240;
    private boolean launchLeft=false;
    private boolean launchRight=false;
    private boolean launch=false;
    private double launchVel=0;
    private boolean correctConfig=false; //checks if artifacts are in correct configuration
    private boolean overflow=false; //checks if there are too many artifacts in one side
    private boolean motifMode=false; //decides whether robot launches for efficiency or motifs
    private boolean eject=false;
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
        transferChanneler = new transferChanneler(hardwareMap);
        colorSensors = new colorSensors(hardwareMap);

        //init mechs
        limelight.init();
        limelight.setPipeline(3);
        flywheel.init();
        ballKickers.retractRight();
        ballKickers.retractLeft();
        transferChanneler.center();
        balls=colorSensors.getBalls();



        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
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



            //error correction logic
            if(!motifMode){
                if(colorSensors.sideSums()[0]+colorSensors.sideSums()[1]>3){
                    correctConfig=false; //if theres more than 3 artfacts in robot
                    eject=true;
                } else{
                    correctConfig=true;
                }
                if(colorSensors.sideSums()[0]>2 || colorSensors.sideSums()[1]>2){
                    correctConfig=false; //if theres 3 artifacts on one side
                    overflow=true;
                    eject=true;
                } else{
                    correctConfig=true;
                    overflow=false;
                }
            }


            //intake
            if(!overflow) {
                intake.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
            } else{
                intake.setPower(0);
            }
            //launch logic
            if(gamepad2.right_bumper && !eject){
                launch=true;
            }
            if(eject){
                if(colorSensors.sideSums()[0]>colorSensors.sideSums()[1]){
                    //left
                }else{
                    //right
                }
            }
            if(launch){
                balls=colorSensors.getBalls();
                if(motifMode){
                    //add code later
                }else{
                    if(colorSensors.sideSums()[0]>colorSensors.sideSums()[1]){
                        //left right left
                    }else{
                        //right left right
                    }
                }
            }





            //Launcher decision making

            //manual
            /*if(gamepad2.right_bumper){
                launchRight=true;
            }
            if(gamepad2.left_bumper){
                launchLeft=true;
            }
            if(gamepad2.back){
                launchLeft=false;
                launchRight=false;
            }
            */
            //Flywheel velocity logic
            if(launchRight||launchLeft){
                if(gamepad2.dpad_up){
                    launchVel=ejectVel;
                }
                else if(limelight.getDistance()==-1){
                    launchVel=defaultLaunchVel;
                }else{

                    launchVel=lut.get(limelight.getDistance());
                }
            }
            else{
                launchVel=0;
            }

            //Kicker logic
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
