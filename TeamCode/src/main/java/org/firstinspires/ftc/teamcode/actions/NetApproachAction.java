package org.firstinspires.ftc.teamcode.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.ErasmusRobot;

public class NetApproachAction implements Action {
    private boolean cancelled = false ;
    private boolean started = false ;
    private ErasmusRobot robot ;

    public NetApproachAction(ErasmusRobot newRobot) {
        robot = newRobot ;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//        if (cancelled) {
//            // TODO: Add logic to stop the motor and brake
//            return false ;
//        }
//        else if (!started) {
//            started = true ;
//            robot.goHigh() ;
//        }
//        else if (robot.atTarget()) cancel() ;

        return true ;
    }

    public void cancel() {
        cancelled = true ;
    }
}
