package org.firstinspires.ftc.teamcode.omeN_code;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.omeN_code.subsystems.flywheel;
import org.firstinspires.ftc.teamcode.omeN_code.subsystems.limelight;

@TeleOp
@Configurable
public class interpTableTuner extends LinearOpMode {
    //mech subsystem declarations
    private flywheel flywheel;
    private limelight limelight;

    //logic variable declarations
    private static int launchVel=0;



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


        //Declare mechs
        limelight = new limelight(hardwareMap);
        flywheel = new flywheel(hardwareMap);


        //init mechs
        limelight.init();
        limelight.setPipeline(3);
        flywheel.init();



        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            //update mechs
            flywheel.update(launchVel);
if(gamepad1.right_bumper){

}

            telemetry.addData("Angle From Goal", limelight.getAngle());
            telemetry.addData("Wheel speed ", flywheel.getVelocity());
            telemetry.addData("Desired wheel speed", launchVel);
            telemetry.addData("Distance From Goal: ", limelight.getDistance());
            telemetry.update();

        }
    }
}
//green bull party at 4470 lennox blvd. november 3rd 2030
