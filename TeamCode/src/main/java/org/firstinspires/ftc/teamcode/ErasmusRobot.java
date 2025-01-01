package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.acmerobotics.roadrunner.Pose2d;

@Config
public class ErasmusRobot {
    OpMode opMode ;
    public DcMotorEx liftMotor;
    public DcMotorEx armMotor;
    public Servo gripperServo ;
    public DcMotor intakeMotor;
    public Servo intakeServo ;
    public Servo armServo ;

    public MecanumDrive drive ;

    // Setpoints =============================================
    // Arm --------------------------------------
    public static int ARM_TARGET = 0 ;
    public static int ARM_MATCH_START = 1300;
    public static int ARM_FLOOR = 10;
    public static int ARM_NET = 1000 ;  //almost straight up
    public static int ARM_CLIP = 1200;
    public static int ARM_INTAKE = 1300 ;
    public static int ARM_HP = 1300;
    public static int ARM_CLIP_INTAKE = -470 ;
    public static int ARM_L1_ASCENSION = 1200;
    public static double ARM_P_COEFFICIENT = 7 ;
    public int armOffset = 0 ;

    public static double ARM_SERVO_POSITION = 0.5 ;
    public static double ARM_SERVO_FLOOR = 0.08 ;
    public static double ARM_SERVO_NET = 0.55 ;
    public static double ARM_SERVO_CLIP = 0.6 ;
    public static double ARM_SERVO_MATCHSTART = 0.6 ;
    public static double ARM_SERVO_INTAKE = 0.7 ;
    // Lift ---------------------------------------
    public static int LIFT_TARGET = 0 ;
    public static int LIFT_START = 0 ; //not used
    public static int LIFT_FLOOR = 0;
    public static int LIFT_NET = 2800 ;   // 43 in high
    public static int LIFT_CLIP = 2900 ;  // 26 in high
    public static int LIFT_CLIPPED = 900;
    public static int LIFT_INTAKE = 500 ;
    public static int LIFT_SUBMERSIBLE = 425 ;
    public static int LIFT_CLIP_INTAKE = 380 ;
    public static double LIFT_P_COEFFICIENT = 8 ;
    // Gripper -----------------------------------
    public static double GRIPPER_TARGET = 0.5 ;
    public static double GRIPPER_OPEN = 0.8 ;
    public static double GRIPPER_CLOSE = 0.4 ;
    public static double GRIPPER_PAUSE_TIME = 0.5 ;
    public boolean gripperOpenState = false ;
    // Intake-----------------------------------
    public static double INTAKE_TARGET = 0.5 ;
    public static double  INTAKE_EXTEND = .01;
    public static double INTAKE_RETRACT = .7;
    public static double INTAKE_RELEASE = .55;
    // Actions -------------------------------
    public Action goToNetPosition ;
    public Action goToClipPosition ;
    public Action goToFloorPosition ;
    public Action payloadToClip ;
    public Action payloadToMatchPosition ;



    public ErasmusRobot(OpMode newOpMode, Pose2d startPose) {
        opMode = newOpMode ;
        drive = new SparkFunOTOSDrive(opMode.hardwareMap, startPose) ;
        // Instantiate arm --------------------------------------------
        armMotor = opMode.hardwareMap.get(DcMotorEx.class, "armMotor");
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setPositionPIDFCoefficients(ARM_P_COEFFICIENT);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setPower(.9);

        armServo = opMode.hardwareMap.get(Servo.class, "armServo");

        // Instantiate lift --------------------------------------------
        liftMotor = opMode.hardwareMap.get(DcMotorEx.class, "liftMotor");
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setPositionPIDFCoefficients(LIFT_P_COEFFICIENT);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setPower(.9);
        liftMotor.setCurrentAlert(3.5, CurrentUnit.AMPS);
        liftMotor.setTargetPositionTolerance(3);
        // Instantiate Servo ------------------------------------------
        gripperServo = opMode.hardwareMap.get(Servo.class, "gripperServo");
        // TODO: Should we set the gripper or let it stay as set??
        // Instantiate Intake
        intakeServo = opMode.hardwareMap.get(Servo.class, "intakeServo");
        intakeServo.setPosition(INTAKE_RETRACT);
        intakeMotor = opMode.hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Instantiate actions -----------------------------------------

        goToClipPosition = new SequentialAction(
                new GripperAction(false),
                new LiftArmAction(LIFT_CLIP, ARM_CLIP+armOffset) ) ;

        payloadToClip = new SequentialAction( // TODO: Incomplete
                new GripperAction(false),
                new SleepAction(1),
                new LiftArmAction(LIFT_FLOOR, ARM_FLOOR+armOffset) ) ;
    }

    public ErasmusRobot(OpMode newOpMode) {
        this(newOpMode, new Pose2d(0,0,Math.toRadians(0))) ;
    }

    public Action goToNetPosition() {
        return new SequentialAction(
                new GripperAction(false),
//                new LiftArmAction(LIFT_NET, ARM_NET+armOffset) ) ;
                new LiftArmAction(LIFT_NET, ARM_SERVO_NET) ) ;
    }
    public Action goToFloorPosition() {
        return new SequentialAction(
                new GripperAction(true),
//                new LiftArmAction(LIFT_FLOOR, ARM_FLOOR + armOffset));
                new LiftArmAction(LIFT_FLOOR, ARM_SERVO_FLOOR));
    }
    public Action goToClipPosition() {
        return new SequentialAction(
                new GripperAction(false),
//                new LiftArmAction(LIFT_CLIP, ARM_CLIP + armOffset));
                new LiftArmAction(LIFT_CLIP, ARM_SERVO_CLIP));
    }

    public void intakeDeploy(){
        intakeMotor.setPower(1);
        intakeServo.setPosition(INTAKE_EXTEND);
    }
    public void intakeRetract(){
        intakeMotor.setPower(0);
        intakeServo.setPosition(INTAKE_RETRACT);
    }
    public void intakeRelease(){
        intakeServo.setPosition(INTAKE_RELEASE);
        intakeMotor.setPower(-1);
    }

    public void setPayloadAll(int newLiftPosition, int newArmPosition, boolean newGripperOpen) {
        liftMotor.setTargetPosition(newLiftPosition);
        armMotor.setTargetPosition(newArmPosition);
        gripperOpenState = newGripperOpen ;
        updateGripper() ;
    }

    public void resetLiftArm() {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setTargetPositionTolerance(50);
    }

    public void liftToPosition(int targetTicks) {
        if (targetTicks > 0) {
            liftMotor.setTargetPosition(targetTicks);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(.9);
            liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }


    public void liftMotorResting() {
        if (liftMotor.getTargetPosition()<1 &&
                liftMotor.isOverCurrent() &&
                //liftMotor.getCurrentPosition()<30 &&
                Math.abs(liftMotor.getVelocity()) < 10) {
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor.setTargetPosition(0);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(0) ;
        }
        else liftMotor.setPower(0.9);
    }


    // Try to always use this to open and close the gripper because it helps with logic elsewhere
    public void updateGripper() {
        if (gripperOpenState) gripperServo.setPosition(GRIPPER_OPEN);
        else gripperServo.setPosition(GRIPPER_CLOSE);
    }

    public void toggleGripper() {
        gripperOpenState =!gripperOpenState ;
        updateGripper() ;
    }

//    public void raiseArm(){
//        armMotor.setTargetPosition(armMotor.getCurrentPosition()+20);
//    }
//    public void lowerArm(){
//        armMotor.setTargetPosition(armMotor.getCurrentPosition()-20);
//    }
    public void tweakLift(int liftAmount) {liftMotor.setTargetPosition(liftMotor.getTargetPosition()+liftAmount) ;  }
    public void tweakArm(int armAmount) {armMotor.setTargetPosition(armMotor.getTargetPosition()+armAmount) ;  }

    // ACTIONS =========================================================
    private class LiftArmAction implements Action {
        private boolean started = false ;
        int liftTarget ;
        int armTarget ;
        double armServoTarget ;
        double startTime ;
        public LiftArmAction(int newLiftTarget, int newArmTarget) {
            startTime = opMode.getRuntime() ;
            liftTarget = newLiftTarget ;
            armTarget = newArmTarget ;
        }
        public LiftArmAction(int newLiftTarget, double newArmTarget) {
            this(newLiftTarget, 0) ;
            armServoTarget = newArmTarget ;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!started) {
                started = true ;
                liftMotor.setTargetPosition(liftTarget);
                //armMotor.setTargetPosition(armTarget+armOffset);
                armServo.setPosition(armServoTarget);
                return true ;
            }
            else if (Math.abs(liftTarget-liftMotor.getCurrentPosition()) < 40 &&
                    Math.abs(armTarget-armMotor.getCurrentPosition()) < 80)
                return false ;
            else if ((opMode.getRuntime() > startTime+3) && liftTarget==0) {
                //resetLiftArm() ;
                return false ;
            }
            return true ;
        }
    }

    private class GripperAction implements Action {
        boolean gripperToOpen;
        double startTime ;
        boolean alreadyThere = false ;
        public GripperAction(boolean newGripperToOpen) {
            startTime = opMode.getRuntime() ;
            gripperToOpen = newGripperToOpen;
            alreadyThere = (gripperToOpen==gripperOpenState) ;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            gripperOpenState = gripperToOpen ;
            updateGripper();
            if (!alreadyThere && opMode.getRuntime() < startTime+GRIPPER_PAUSE_TIME) return true ;
            else return false ;
        }
    }
}




