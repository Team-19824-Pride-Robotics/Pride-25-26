package org.firstinspires.ftc.teamcode.V2.subsystems;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.FeedbackServoEx;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

public class BallKickers {
    public static final BallKickers INSTANCE = new BallKickers();
    private BallKickers() { }
    public static double downLeftPosition = 0.8;
    public static double upLeftPosition = 0.6;
    public static double downRightPosition = 0.85;
    public static double upRightPosition = 0.45;
    FeedbackServoEx leftBallKicker = new FeedbackServoEx("lBKE", "lBK", 0.01);
    FeedbackServoEx rightBallKicker = new FeedbackServoEx("rBKE", "rBK", 0.01);
    private final ControlSystem flywheelControlSystem = ControlSystem.builder()
            .velPid(0.005, 0, 0)
            .build();


    public void initialize() {
        retractLeft();
        retractRight();
    }


    public void kickLeft() {
        new SetPosition(leftBallKicker, upLeftPosition).schedule();
    }
    public void kickRight() {
        new SetPosition(rightBallKicker, upRightPosition).schedule();
    }
    public void retractLeft() {
        new SetPosition(leftBallKicker, downLeftPosition).schedule();
    }
    public void retractRight() {
        new SetPosition(rightBallKicker, downRightPosition).schedule();
    }
    public double getLeftPos(){
        return leftBallKicker.getCurrentPosition();
    }
    public double getRightPos(){
        return rightBallKicker.getCurrentPosition();
    }

}
