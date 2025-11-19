package org.firstinspires.ftc.teamcode.prideRobotics.jacksonTunerz;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.prideRobotics.subsystems.colorSensors;

@TeleOp
@Configurable
public class colorSensorTuner extends LinearOpMode {
    private colorSensors colorSensors;
    private NormalizedRGBA[] colors;
    @Override
    public void runOpMode() throws InterruptedException {

        colorSensors = new colorSensors(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            colors=colorSensors.getColorsBro();

            telemetry.addData("lLRed ", colors[0].red);
            telemetry.addData("lLGreen", colors[0].green);
            telemetry.addData("lLBlue", colors[0].blue);

            telemetry.addLine();

            telemetry.addData("rLRed ", colors[1].red);
            telemetry.addData("rLGreen", colors[1].green);
            telemetry.addData("rLBlue", colors[1].blue);

            telemetry.addLine();

            telemetry.addData("lRRed ", colors[2].red);
            telemetry.addData("lRGreen", colors[2].green);
            telemetry.addData("lRBlue", colors[2].blue);

            telemetry.addLine();

            telemetry.addData("rRRed ", colors[3].red);
            telemetry.addData("rRGreen", colors[3].green);
            telemetry.addData("rRBlue", colors[3].blue);
            telemetry.update();
        }
    }
}
//stop copying my code, ftc eric/ftc ivan fartenberry
