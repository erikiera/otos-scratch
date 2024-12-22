package org.firstinspires.ftc.teamcode.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class CancelableAction implements Action {
    private boolean cancelled = false ;

    public CancelableAction() {

    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (cancelled) {
            return false ;
        }
        return true ;
    }

    public void cancel() {
        cancelled = true ;
    }
}
