package org.firstinspires.ftc.teamcode.V1.jacksonTunerz;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;


import org.firstinspires.ftc.teamcode.V1.subsystems.intake;
import org.firstinspires.ftc.teamcode.V1.subsystems.flywheel;
import org.firstinspires.ftc.teamcode.V1.subsystems.ballKickers;
import org.firstinspires.ftc.teamcode.V1.subsystems.limelight;
import org.firstinspires.ftc.teamcode.V1.subsystems.colorSensors;
@TeleOp
@Configurable
public class  DrivetrainTest extends LinearOpMode {
    //mech subsystem declarations



    @Override
    public void runOpMode() throws InterruptedException {



        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("fLD");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("bLD");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("fRD");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("bRD");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);







        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if(gamepad1.a){
                backRightMotor.setPower(1);
            } else{
                backRightMotor.setPower(0);
            }
            if(gamepad1.b){
                frontRightMotor.setPower(1);
            } else{
                frontRightMotor.setPower(0);
            }
            if(gamepad1.y){
                frontLeftMotor.setPower(1);
            } else{
                frontLeftMotor.setPower(0);
            }
            if(gamepad1.x){
                backLeftMotor.setPower(1);
            } else{
                backLeftMotor.setPower(0);
            }
        }
    }
}

