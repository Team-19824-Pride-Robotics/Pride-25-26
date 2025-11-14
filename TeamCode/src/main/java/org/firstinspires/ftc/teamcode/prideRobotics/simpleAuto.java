package org.firstinspires.ftc.teamcode.prideRobotics;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.prideRobotics.subsystems.intake;
import org.firstinspires.ftc.teamcode.prideRobotics.subsystems.flywheel;
import org.firstinspires.ftc.teamcode.prideRobotics.subsystems.ballKickers;
import org.firstinspires.ftc.teamcode.prideRobotics.subsystems.limelight;
import org.firstinspires.ftc.teamcode.prideRobotics.subsystems.transferChanneler;

@Autonomous
@Configurable
public class simpleAuto extends LinearOpMode {
double seconds=0;
private static double time = 3;
private static double power = 0.5;
private static double launchPower = 1300;
    private static double UpRightPos=180;
    private static double UpLeftPos=240;
    private flywheel flywheel;
    private ballKickers ballKickers;
    private limelight limelight;
    @Override
    public void runOpMode() throws InterruptedException {

        // Declare motor ok
        // Absolutely yes make ID's match configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("fLD");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("bLD");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("fRD");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("bRD");

        //reverse drive motors
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        ElapsedTime timer = new ElapsedTime();

        limelight = new limelight(hardwareMap);
        flywheel = new flywheel(hardwareMap);
        ballKickers = new ballKickers(hardwareMap);

        limelight.init();
        limelight.setPipeline(3);
        flywheel.init();
        ballKickers.retractRight();
        ballKickers.retractLeft();

        waitForStart();

        if (isStopRequested()) return;
        flywheel.update(launchPower);
        while(!(flywheel.getVelocity() < launchPower+40 && flywheel.getVelocity() > launchPower-40)){
            flywheel.update(launchPower);
            telemetry.addData("launch vel", flywheel.getVelocity());
            telemetry.addData("set vel", launchPower);
            telemetry.update();

        }
        ballKickers.kickLeft();
        ballKickers.update();
        while(!(ballKickers.getLeftPos()<UpLeftPos)){
            idle();
        }
        flywheel.update(launchPower);
        ballKickers.retractLeft();
        ballKickers.update();
        while(!(flywheel.getVelocity() < launchPower+40 && flywheel.getVelocity() > launchPower-40)){
            flywheel.update(launchPower);
        }
        ballKickers.kickRight();
        ballKickers.update();
        while(!(ballKickers.getRightPos()>UpRightPos)){
            flywheel.update(launchPower);
        }
        ballKickers.retractRight();
        ballKickers.update();
        while(!(flywheel.getVelocity() < launchPower+40 && flywheel.getVelocity() > launchPower-40)){
            flywheel.update(launchPower);
        }
        ballKickers.kickLeft();
        ballKickers.update();
        while(!(ballKickers.getLeftPos()<UpLeftPos)){
            idle();
        }
        flywheel.update(0);
        ballKickers.retractLeft();
        ballKickers.update();
        //I know this is not how i should do it but whatever

        timer.reset();
        timer.startTime();

        while(seconds<time){
            seconds=timer.seconds();
            frontLeftMotor.setPower(power);
            frontRightMotor.setPower(power);
            backLeftMotor.setPower(power);
            backRightMotor.setPower(power);
        }
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

    }
}
//green bull party at 4470 lennox blvd. november 3rd 2030
