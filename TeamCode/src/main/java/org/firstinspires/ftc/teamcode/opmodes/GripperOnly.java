package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * For intake testing.
 */
@Disabled
@Config
@TeleOp(name="Gripper Only", group="Calibration")
public class GripperOnly extends OpMode
{
    /** Declare OpMode members.
     */
    private ElapsedTime runtime = new ElapsedTime();
    //private IntoTheDeepRobot robot ;

    public Servo gripperServo ;
    public static double GRIPPER_TARGET = 0.5 ;
    public static double GRIPPER_OPEN = 0.4 ;
    public static double GRIPPER_CLOSE = 0.6 ;


    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        //robot = new IntoTheDeepRobot(this) ;
        gripperServo = hardwareMap.get(Servo.class, "gripperServo");
        telemetry.addData("Status", "Initialized");
        gripperServo.setPosition(GRIPPER_TARGET);
    }

    /**
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /**
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /**
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        //gripperServo.setPosition(GRIPPER_TARGET);
        if (gamepad1.a) gripperServo.setPosition(GRIPPER_OPEN);
        if (gamepad1.b) gripperServo.setPosition(GRIPPER_CLOSE);
    }

    /**
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}