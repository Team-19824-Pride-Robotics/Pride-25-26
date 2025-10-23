package org.firstinspires.ftc.teamcode.prideRobotics;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;


import org.firstinspires.ftc.teamcode.prideRobotics.subsystems.intake;
import org.firstinspires.ftc.teamcode.prideRobotics.subsystems.flywheel;
import org.firstinspires.ftc.teamcode.prideRobotics.subsystems.ballKickers;
import org.firstinspires.ftc.teamcode.prideRobotics.subsystems.limelight;
import org.firstinspires.ftc.teamcode.prideRobotics.subsystems.transferChanneler;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
@Configurable
public class blueTeleop extends LinearOpMode {
//mech subsystem declarations
    private intake intake;
    private flywheel flywheel;
   private ballKickers ballKickers;
    private limelight limelight;
    private transferChanneler transferChanneler;
//logic variable declarations
    private boolean launchLeft=false;
    private boolean launchRight=false;

    //flywheel setup
    private static double kP=0;
    private static double kI=0;
    private static double kD=0;
    private static double kF=0;
    PIDFController pidf = new PIDFController(kP, kI, kD, kF);
    InterpLUT lut = new InterpLUT();


    @Override
    public void runOpMode() throws InterruptedException {
        //Setup interp table

        //Adding each val with a key
        lut.add(1.1, 0.2); //todo: set values, these are placeholers.

        //generating final equation
        lut.createLUT();

        // Declare motor ok
        // Absolutely yes make ID's match configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("lF");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("lB");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rF");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rB");

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
        flywheel.init();
        ballKickers.retractRight();
        ballKickers.retractLeft();
        transferChanneler.center();



        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            //drive control
            double y = -gamepad2.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad2.left_stick_x;
            double rx = gamepad2.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(-frontLeftPower * (1-gamepad1.right_trigger));
            backLeftMotor.setPower(-backLeftPower * (1-gamepad1.right_trigger));
            frontRightMotor.setPower(frontRightPower* (1-gamepad1.right_trigger));
            backRightMotor.setPower(-backRightPower* (1-gamepad1.right_trigger));

            //////////////
            //Mechansims//
            //////////////

            //intake
                intake.setPower(gamepad2.right_trigger-gamepad2.left_trigger);

            //flywheel

            //power calculations
            if(limelight.getDistance()==-1){
                pidf.setSetPoint(1000); //Todo: Set to a low enough power to maintain stable voltage. Current val is a placeholder
                flywheel.update(pidf.calculate());
            }else{
                pidf.setSetPoint(lut.get(limelight.getDistance()));
                flywheel.update(pidf.calculate());
            }

            if(gamepad2.right_bumper){
                launchRight=true;
            }
            if(gamepad2.left_bumper){
                launchLeft=true;
            }



            telemetry.addData("Wheel speed ", flywheel.getVelocity());
            telemetry.addData("Desired wheel speed", launchVel);
            telemetry.addData("Distance From Goal: ", distanceFromGoal);
            telemetry.addData("Angle From Goal", angleFromGoal);
            telemetry.update();

        }
    }
}
//green bull party at 4470 lennox blvd. november 3rd 2030
