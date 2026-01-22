package org.firstinspires.ftc.teamcode.omeN_code;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.omeN_code.subsystems.intake;
import org.firstinspires.ftc.teamcode.omeN_code.subsystems.flywheel;
import org.firstinspires.ftc.teamcode.omeN_code.subsystems.ballKickers;
import org.firstinspires.ftc.teamcode.omeN_code.subsystems.limelight;

@Autonomous
@Configurable
public class simpleAuto extends LinearOpMode {
double seconds=0;
double motif=0;
private static double secondLaunchWait = 3;
private static double thirdLaunchWait = 4;
private static double driveWait = 3;
private static double driveTime = 1;
private static double power = 0.5;
private static double launchPower = 1280;
    private static double UpRightPos=180;
    private static double UpLeftPos=240;
    private double DownLeftPos=114;
    private double DownRightPos=309;
    private flywheel flywheel;
    private ballKickers ballKickers;
    private limelight limelight;
    private intake intake;
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
        intake = new intake(hardwareMap);

        limelight.init();
        limelight.setPipeline(0);
        flywheel.init();
        ballKickers.retractRight();
        ballKickers.retractLeft();

        waitForStart();

        if (isStopRequested()) return;
        //Scan obelisk
        motif=limelight.scanAuto();

        //Launch first artifact
        flywheel.update(launchPower);
        while(!(flywheel.getVelocity() < launchPower+40 && flywheel.getVelocity() > launchPower-40)){
            flywheel.update(launchPower);
            telemetry.addData("launch vel", flywheel.getVelocity());
            telemetry.addData("set vel", launchPower);
            telemetry.update();

        }
        if(motif==1 || motif==2){
            ballKickers.kickLeft();
            ballKickers.update();
            while(!(ballKickers.getLeftPos()<UpLeftPos)){
                flywheel.update(launchPower);
                idle();
            }
            ballKickers.retractLeft();
            ballKickers.update();
        }
        else{
            ballKickers.kickRight();
            ballKickers.update();
            while(!(ballKickers.getRightPos()>UpRightPos)){
                flywheel.update(launchPower);
                idle();
            }
            ballKickers.retractRight();
            ballKickers.update();
        }

        //launch second artifact
        flywheel.update(launchPower);
        while(!(flywheel.getVelocity() < launchPower+40 && flywheel.getVelocity() > launchPower-40)){
            flywheel.update(launchPower);
        }
        timer.reset();
        timer.startTime();
        seconds=timer.seconds();
        if(motif==0 || motif==2){
            while(!(ballKickers.getLeftPos()>DownLeftPos) && (seconds = timer.seconds())<secondLaunchWait){
                seconds=timer.seconds();
                flywheel.update(launchPower);
            }
            ballKickers.kickLeft();
            ballKickers.update();
            while(!(ballKickers.getLeftPos()<UpLeftPos)){
                seconds=timer.seconds();
                flywheel.update(launchPower);
            }
            ballKickers.retractLeft();
            ballKickers.update();
        }
        else{
            while(!(ballKickers.getRightPos()<DownRightPos) && (seconds = timer.seconds())<secondLaunchWait){
                seconds=timer.seconds();
                flywheel.update(launchPower);
            }
            ballKickers.kickRight();
            ballKickers.update();
            while(!(ballKickers.getRightPos()>UpRightPos)){
                seconds=timer.seconds();
                flywheel.update(launchPower);
                idle();
            }
            ballKickers.retractRight();
            ballKickers.update();
        }

        //launch third artifact
        timer.reset();
        timer.startTime();
        seconds=timer.seconds();
        while(!(flywheel.getVelocity() < launchPower+40 && flywheel.getVelocity() > launchPower-40)){
            seconds=timer.seconds();
            intake.setPower(-1);
            intake.update();
            flywheel.update(launchPower);

        }

        if(motif==0 || motif==1){

            while(!(ballKickers.getLeftPos()>DownLeftPos) && (seconds = timer.seconds())<thirdLaunchWait){
                seconds=timer.seconds();
                flywheel.update(launchPower);
                intake.setPower(-1);
                intake.update();
            }
            ballKickers.kickLeft();
            ballKickers.update();
            while(!(ballKickers.getLeftPos()<UpLeftPos)){
                flywheel.update(launchPower);
                idle();
            }
            ballKickers.retractLeft();
            ballKickers.update();
        }
        else{
            while(!(ballKickers.getRightPos()<DownRightPos) && (seconds = timer.seconds())<thirdLaunchWait){
                seconds=timer.seconds();
                flywheel.update(launchPower);
                intake.setPower(-1);
                intake.update();
            }
            ballKickers.kickRight();
            ballKickers.update();
            while(!(ballKickers.getRightPos()>UpRightPos)){
                flywheel.update(launchPower);
                idle();
            }
            ballKickers.retractRight();
            ballKickers.update();
        }
        intake.setPower(0);
        flywheel.update(0);
        timer.reset();
        timer.startTime();
        seconds=timer.seconds();
        while((seconds = timer.seconds())<driveWait){
            seconds=timer.seconds();
            idle();
        }

//exit start zone, twin
        timer.reset();
        timer.startTime();
        seconds=timer.seconds();
        while((seconds = timer.seconds())<driveTime){
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
//hey roar, stop stealing my code 😡😡😡😡😡😡😡😡







//jk idc 😊
