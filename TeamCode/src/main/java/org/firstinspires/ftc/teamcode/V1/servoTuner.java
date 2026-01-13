package org.firstinspires.ftc.teamcode.V1;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Configurable
public class servoTuner extends LinearOpMode {
    //mech subsystem declarations
    private Servo servo;
    private DcMotor flywheel;
    private DcMotor flywheelB;
    private AnalogInput servoE;
    //logic variable declarations
    private static double down = 0;
    private static double up = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        servo = hardwareMap.get(Servo.class, "servo");
        servoE = hardwareMap.get(AnalogInput.class, "E");

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.a) {
                servo.setPosition(down);
            }
            if (gamepad1.b) {
                servo.setPosition(up);
            }

            telemetry.addData("Servo pos: ", servoE.getVoltage() / 3.3 * 360);
            telemetry.update();
        }
//green bull party at 4470 lennox blvd. november 3rd 2030
    }
}