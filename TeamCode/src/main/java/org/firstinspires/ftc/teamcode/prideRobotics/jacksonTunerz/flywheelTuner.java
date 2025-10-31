package org.firstinspires.ftc.teamcode.prideRobotics.jacksonTunerz;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.prideRobotics.subsystems.colorSensors;
import org.firstinspires.ftc.teamcode.prideRobotics.subsystems.flywheel;

@TeleOp
@Configurable
public class flywheelTuner extends LinearOpMode {
    private flywheel flywheel;
    private int launchPower=0;
    public static int tempVal=200;

    @Override
    public void runOpMode() throws InterruptedException {


        flywheel = new flywheel(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if(gamepad1.a){
                launchPower=tempVal;
            }
            flywheel.update(launchPower);

            telemetry.addData("Set Velocity:", launchPower);
            telemetry.addData("Velocity: ", flywheel.getVelocity());
            telemetry.update();

        }
    }
}
//stop copying my code, ftc eric/ftc ivan fartenberry
