package org.firstinspires.ftc.teamcode.prideRobotics;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.prideRobotics.subsystems.intake;
import org.firstinspires.ftc.teamcode.prideRobotics.subsystems.flywheel;
import org.firstinspires.ftc.teamcode.prideRobotics.subsystems.ballKickers;
import org.firstinspires.ftc.teamcode.prideRobotics.subsystems.limelight;
import org.firstinspires.ftc.teamcode.prideRobotics.subsystems.transferChanneler;

@TeleOp
@Configurable
public class servoTuner extends LinearOpMode {
    //mech subsystem declarations
    private Servo servo;
    private DcMotor flywheel;
    private DcMotor flywheelB;
    //logic variable declarations
    private static double down = 0;
    private static double up = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        servo = hardwareMap.get(Servo.class, "servo");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        flywheelB = hardwareMap.get(DcMotor.class, "flywheelB");


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.a) {
                servo.setPosition(down);
            }
            if (gamepad1.b) {
                servo.setPosition(up);
            }
            if (gamepad1.right_bumper) {
                flywheel.setPower(1);
                flywheelB.setPower(1);
            }
            if (gamepad1.left_bumper) {
                flywheel.setPower(-1);
                flywheelB.setPower(-1);
            }
            if (!gamepad1.left_bumper && !gamepad1.right_bumper) {
                flywheel.setPower(0);
                flywheelB.setPower(0);
            }
        }
//green bull party at 4470 lennox blvd. november 3rd 2030
    }
}