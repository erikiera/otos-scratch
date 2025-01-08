package org.firstinspires.ftc.teamcode.calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;

@Config
@TeleOp(name="Payload Only", group="Calibration")
@Disabled
public class PayloadOnly extends LinearOpMode {
    // Declare objects and variables
    DcMotor liftMotor;
    DcMotor armMotor;
    public Servo gripperServo ;
    public static int LIFT_TARGET = 0;
    // Setpoints =============================================
    // Lift --------------------------------------
    public static int ARM_START = 0 ;
    public static int ARM_NET = 2300 ;
    public static int ARM_CLIP = 1200 ;
    public static int ARM_INTAKE = 2000 ;
    // Arm ---------------------------------------
    public static int LIFT_START = 0 ;
    public static int LIFT_NET = 3500 ;   // 43 in high
    public static int LIFT_CLIP = 1000 ;  // 26 in high
    public static int LIFT_INTAKE = 500 ;
    // Gripper -----------------------------------
    public static double GRIPPER_OPEN = 0.73 ;
    public static double GRIPPER_CLOSE = 0.41 ;

    @Override
    public void runOpMode() throws InterruptedException {
        // Instantiate object and variable
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(0, 0, 0));
        // Instantiate arm ========================================
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setPower(.8);
        // Instantiate lift ======================================
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setPower(.8);
        // Instantiate Servo ======================================
        gripperServo = hardwareMap.get(Servo.class, "gripperServo");
        gripperServo.setPosition(GRIPPER_OPEN);
        // =========================================================
        telemetry.addLine("Both motors set to positive.") ;
        telemetry.update() ;



        waitForStart();

        while (opModeIsActive()) {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            drive.updatePoseEstimate();

            if (gamepad1.x) drive.otos.setPosition(new SparkFunOTOS.Pose2D(0,0,0));

            if (gamepad1.dpad_down) {
                gripperServo.setPosition(GRIPPER_OPEN) ;
                sleep(200) ;
                armMotor.setTargetPosition(ARM_START);
                liftMotor.setTargetPosition(LIFT_START);
            }
            else if (gamepad1.dpad_up) {
                gripperServo.setPosition(GRIPPER_CLOSE) ;
                sleep(500) ;
                armMotor.setTargetPosition(ARM_NET);
                liftMotor.setTargetPosition(LIFT_NET);
            }
            else if (gamepad1.dpad_right) {
                armMotor.setTargetPosition(ARM_CLIP);
                liftMotor.setTargetPosition(LIFT_CLIP);
            }
            else if (gamepad1.dpad_left) {
                armMotor.setTargetPosition(ARM_INTAKE);
                liftMotor.setTargetPosition(LIFT_INTAKE);
            }
            else if (gamepad1.b) {
                gripperServo.setPosition(GRIPPER_OPEN);
            }
            else if (gamepad1.a) {
                gripperServo.setPosition(GRIPPER_CLOSE);
            }



            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}
