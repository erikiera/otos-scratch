package org.firstinspires.ftc.teamcode.opmodes;

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

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.ErasmusRobot;
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;

@Config
@TeleOp(name="Arm Only", group="Calibration")
//@Disabled
public class ArmOnly extends LinearOpMode {
    // Declare objects and variables
    public static int ARM_TARGET = 0;
    public static double ARM_P_COEFFICIENT = 0.5 ;
    ErasmusRobot robot ;

    @Override
    public void runOpMode() throws InterruptedException {
        // Instantiate object and variable
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new ErasmusRobot(this) ;

        waitForStart();

        while (opModeIsActive()) {
            robot.drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            robot.drive.updatePoseEstimate();


            if (gamepad1.y) robot.armMotor.setTargetPosition(ARM_TARGET);
            if (gamepad1.dpad_up) robot.armMotor.setTargetPosition(robot.ARM_NET);
            if (gamepad1.dpad_down) robot.armMotor.setTargetPosition(robot.ARM_FLOOR);
            if (gamepad1.b) robot.armMotor.setPositionPIDFCoefficients(ARM_P_COEFFICIENT) ;

            if (gamepad1.right_trigger>0.2) robot.tweakArm((int)(gamepad1.right_trigger*10)) ;
            if (gamepad1.left_trigger>0.2) robot.tweakArm((int)(-gamepad1.left_trigger*10)) ;

            telemetry.addData("Arm Ticks" , robot.armMotor.getCurrentPosition()) ;
            telemetry.addData("x", robot.drive.pose.position.x);
            telemetry.addData("y", robot.drive.pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(robot.drive.pose.heading.toDouble()));
            telemetry.addData("Arm P Coeff", robot.armMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION)) ;
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), robot.drive.pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}
