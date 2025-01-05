package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.ErasmusRobot;

import java.util.Objects;

@Config
@TeleOp(name="Mentor Teleop", group="Teleop")
public class MentorTeleop extends LinearOpMode {
    // Declare objects and variables
    ErasmusRobot robot ;
    public static double SPEEDMULTIPLE = 0.8 ;
    public static double TURNMULTIPLE = 0.5 ;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new ErasmusRobot(this) ;
        Action currentAction = null ;
        TelemetryPacket packet = new TelemetryPacket();
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        GamepadEx gamepadEx2 = new GamepadEx(gamepad2);

        // Instantiate object and variable
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Motors are initialized.") ;
        telemetry.update() ;

        waitForStart();
        resetRuntime();
        // ===================== Loop for remainder of OpMode =========================================
        while (opModeIsActive()) {
            robot.drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y*SPEEDMULTIPLE,
                            -gamepad1.left_stick_x*(SPEEDMULTIPLE)
                    ),
                    -gamepad1.right_stick_x*TURNMULTIPLE
            ));

            // --------- This runs the current action. If the action is done, it nullifies it ------------
            if (Objects.nonNull(currentAction)) if (!currentAction.run(packet)) currentAction= null ;

            robot.liftMotorResting(); // Prevents overheating in the lift motor

            gamepadEx1.readButtons() ;
            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) currentAction = robot.goToNetPosition();
            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) currentAction = robot.goToFloorPosition();
            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) currentAction = robot.goToClipPosition();
            //if (gamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) robot.clipReset();
            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.X)) robot.toggleGripper();
            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) currentAction = robot.goToIntakeDeploy();
            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) currentAction = robot.goToIntakeRetract();
            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.B)) currentAction = robot.goToIntakeRelease();
//            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.B)) robot.armServo.setPosition(ErasmusRobot.ARM_SERVO_POSITION);
            //if (gamepadEx1.wasJustPressed(GamepadKeys.Button.Y)) {}
            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.BACK)) robot.armServo.setPosition(ErasmusRobot.ARM_SERVO_POSITION);
            if (gamepad1.right_trigger>0.2) robot.tweakArm((int)(gamepad1.right_trigger*10)) ;
            if (gamepad1.left_trigger>0.2) robot.tweakArm((int)(-gamepad1.left_trigger*10)) ;
            
//            if (Math.abs(gamepad2.left_stick_y) > 0.1 ) {
//                robot.tweakLift((int)(-gamepad2.left_stick_y*10)) ;}
//            if (Math.abs(gamepad2.right_stick_y) > 0.1 ) {
//                robot.tweakArm((int)(-gamepad2.right_stick_y*20)) ; }
            gamepadEx2.readButtons() ;
//            if (gamepadEx2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) robot.netApproach();
//            if (gamepadEx2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) robot.netReset();
//            if (gamepadEx2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) robot.clipApproach();
//            if (gamepadEx2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) robot.clipReset();
//            if (gamepadEx2.wasJustPressed(GamepadKeys.Button.X)) robot.gripperToggle();
//            if (gamepadEx2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) robot.intakeDeploy();
//            if (gamepadEx2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) robot.intakeRetract();
//            if (gamepadEx2.wasJustPressed(GamepadKeys.Button.B)) robot.intakeRelease();
//            if (gamepadEx2.wasJustPressed(GamepadKeys.Button.Y)) robot.resetPayload();
//            if (gamepadEx2.wasJustPressed(GamepadKeys.Button.BACK)) robot.toSubmersible();
//            if (gamepad1.right_trigger>0.5) robot.raiseArm();
//            if (gamepad1.left_trigger>0.5) robot.lowerArm();

            telemetry.addData("Axon" , ErasmusRobot.ARM_SERVO_POSITION) ;
            telemetry.addData("Current" , robot.liftMotor.getCurrent(CurrentUnit.AMPS)) ;
            telemetry.addData("Lift Target" , robot.liftMotor.getTargetPosition()) ;
            telemetry.addData("Lift Ticks" , robot.liftMotor.getCurrentPosition()) ;
            telemetry.addData("Motor Velocity" , robot.liftMotor.getVelocity()) ;
            telemetry.addData("Lift Busy", robot.liftMotor.isBusy()) ;
            telemetry.addData("x", robot.drive.pose.position.x);
            telemetry.addData("y", robot.drive.pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(robot.drive.pose.heading.toDouble()));
            telemetry.update();

            //TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}