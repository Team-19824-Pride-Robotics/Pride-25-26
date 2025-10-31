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
public class limelightDistanceThingy extends LinearOpMode {
    //mech subsystem declarations
    private limelight limelight;



    @Override
    public void runOpMode() throws InterruptedException {

        limelight = new limelight(hardwareMap);


        //init mechs
        limelight.init();
        limelight.setPipeline(3);




        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
limelight.init();
limelight.setPipeline(3);





            telemetry.addData("Distance From Goal: ", limelight.getDistance());
            telemetry.update();

        }
    }
}
//green bull party at 4470 lennox blvd. november 3rd 2030
