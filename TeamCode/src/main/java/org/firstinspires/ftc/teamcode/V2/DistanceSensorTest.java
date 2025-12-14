package org.firstinspires.ftc.teamcode.V2;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.V2.subsystems.Intake;
import org.firstinspires.ftc.teamcode.V2.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.V2.subsystems.BallKickers;
import org.firstinspires.ftc.teamcode.V2.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.V2.subsystems.ColorSensors;
import org.firstinspires.ftc.teamcode.V2.subsystems.DistanceSensors;
@TeleOp
@Configurable
public class DistanceSensorTest extends LinearOpMode {
    //mech subsystem declarations

    private DistanceSensors distanceSensors;
    //fun variables





    @Override
    public void runOpMode() throws InterruptedException {



        distanceSensors = new DistanceSensors(hardwareMap);



        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            telemetry.addData("Count", distanceSensors.getCount());
            telemetry.addData("Side", distanceSensors.getSide());
            telemetry.addData("Raw data", distanceSensors.rawVals());

            telemetry.update();

        }
    }
}
//green bull party at 4470 lennox blvd. november 3rd 2030
