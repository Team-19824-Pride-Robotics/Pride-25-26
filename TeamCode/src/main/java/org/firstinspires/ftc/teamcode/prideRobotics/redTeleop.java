package org.firstinspires.ftc.teamcode.prideRobotics;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.firstinspires.ftc.teamcode.prideRobotics.subsystems.intake;
import org.firstinspires.ftc.teamcode.prideRobotics.subsystems.flywheel;
import org.firstinspires.ftc.teamcode.prideRobotics.subsystems.ballKickers;
import org.firstinspires.ftc.teamcode.prideRobotics.subsystems.limelight;
import org.firstinspires.ftc.teamcode.prideRobotics.subsystems.transferChanneler;

@TeleOp
@Configurable
public class redTeleop extends LinearOpMode {
//mech subsystem declarations
    private intake intake;
    private flywheel flywheel;
   private ballKickers ballKickers;
    private limelight limelight;
    private transferChanneler transferChanneler;
//logic variable declarations
    private boolean launchLeft=false;
    private boolean launchRight=false;
    private double launchVel;
    private double UpRightPos=230;
    private double UpLeftPos=185;
    private double DownRightPos=309;
    private double DownLeftPos=114;



    //flywheel setup

    InterpLUT lut = new InterpLUT();


    @Override
    public void runOpMode() throws InterruptedException {
        //Setup interp table

        //Adding each val with a key
        lut.add(1.1, 0.2);
        lut.add(4.0, 0.9);//todo: set values, these are placeholers.

        //generating final equation
        lut.createLUT();

        // Declare motor ok
        // Absolutely yes make ID's match configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("fLD");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("bLD");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("fRD");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("bRD");

        //reverse drive motors
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Declare mechs
        intake = new intake(hardwareMap);
        limelight = new limelight(hardwareMap);
        flywheel = new flywheel(hardwareMap);
        ballKickers = new ballKickers(hardwareMap);
        transferChanneler = new transferChanneler(hardwareMap);

        //init mechs
        limelight.init();
        limelight.setPipeline(3);
        flywheel.init();
        ballKickers.retractRight();
        ballKickers.retractLeft();
        transferChanneler.center();



        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            //drive control
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower * (1-gamepad1.right_trigger));
            backLeftMotor.setPower(backLeftPower * (1-gamepad1.right_trigger));
            frontRightMotor.setPower(frontRightPower* (1-gamepad1.right_trigger));
            backRightMotor.setPower(backRightPower* (1-gamepad1.right_trigger));

            //////////////
            //Mechansims//
            //////////////

            //intake
                intake.setPower(gamepad2.right_trigger-gamepad2.left_trigger);

            //flywheel

            //power calculations
            if(limelight.getDistance()==-1){
                launchVel=600;
            }else{
                launchVel=lut.get(limelight.getDistance());
            }


            //ball kicking
            if(gamepad2.right_bumper){
               launchRight=true;
            }
            if(gamepad2.left_bumper){
                launchLeft=true;
            }
            if(gamepad2.back){
                launchLeft=false;
                launchRight=false;
            }
            if(launchRight||launchLeft){
                if(limelight.getDistance()==-1){
                    launchVel=1000;
                }else{
                    launchVel=lut.get(limelight.getDistance());
                }
            }  else{
                launchVel=600;
            }

            if(launchRight) {
                ballKickers.retractLeft();
                if (flywheel.getVelocity() == launchVel) {
                    ballKickers.kickRight();
                }
            }
            if(launchRight&&ballKickers.getRightPos()<UpRightPos){
                ballKickers.retractRight();
                launchRight=false;
            }

            if(launchLeft) {
                ballKickers.retractRight();
                if (flywheel.getVelocity() == launchVel) {
                    ballKickers.kickLeft();
                }
            }
            if(launchLeft&&ballKickers.getLeftPos()>UpLeftPos){
                ballKickers.retractLeft();
                launchLeft=false;
            }
            //Channeler testing

            if(gamepad2.x){
                transferChanneler.coverLeft();
            }
            if(gamepad2.a){
                transferChanneler.center();
            }
            if(gamepad2.b){
                transferChanneler.coverRight();
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
            telemetry.update();

        }
    }
}
//green bull party at 4470 lennox blvd. november 3rd 2030
