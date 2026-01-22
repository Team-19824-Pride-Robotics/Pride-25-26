package org.firstinspires.ftc.teamcode.omeN_code;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.firstinspires.ftc.teamcode.omeN_code.subsystems.intake;
import org.firstinspires.ftc.teamcode.omeN_code.subsystems.flywheel;
import org.firstinspires.ftc.teamcode.omeN_code.subsystems.ballKickers;
import org.firstinspires.ftc.teamcode.omeN_code.subsystems.limelight;

@TeleOp
@Configurable
public class Testleop extends LinearOpMode {
    //mech subsystem declarations
    private intake intake;
    private flywheel flywheel;
    private ballKickers ballKickers;
    private limelight limelight;
    //logic variable declarations
    private boolean launchLeft=false;
    private boolean launchRight=false;
    private static double launchVel=1000;

    //flywheel setup
    private static double kP=0;
    private static double kI=0;
    private static double kD=0;
    private static double kF=0;
    private static int T=60;
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


        //init mechs
        limelight.init();
        limelight.setPipeline(3);
        flywheel.init();
        ballKickers.retractRight();
        ballKickers.retractLeft();




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

            pidf.setSetPoint(launchVel);

            //ball kicking
            if(gamepad2.right_bumper && launchVel<flywheel.getVelocity()+T && launchVel>flywheel.getVelocity()-T){
                ballKickers.kickRight();
            }
            if(gamepad2.left_bumper && launchVel<flywheel.getVelocity()+T && launchVel>flywheel.getVelocity()-T){
                ballKickers.kickLeft();
            }

            //Channeler testing



            //update mechs
            flywheel.update(pidf.calculate());
            ballKickers.update();
            intake.update();


            telemetry.addData("Angle From Goal", limelight.getAngle());
            telemetry.addData("Wheel speed ", flywheel.getVelocity());
            telemetry.addData("Desired wheel speed", launchVel);
            telemetry.addData("Distance From Goal: ", limelight.getDistance());
            telemetry.update();

        }
    }
}
//green bull party at 4470 lennox blvd. november 3rd 2030
