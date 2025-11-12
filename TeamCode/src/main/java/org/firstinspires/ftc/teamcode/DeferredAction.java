package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import java.util.function.Supplier;

public class DeferredAction implements Action {
    private final Supplier<Action> supplier;
    private Action delegate = null;

    public DeferredAction(Supplier<Action> supplier) {
        this.supplier = supplier;
    }

    @Override
    public boolean run(TelemetryPacket telemetryPacket) {
        if (delegate == null) {
            delegate = supplier.get();
            if (delegate == null) {
                // nothing to run
                return false;
            }
        }
        boolean cont = delegate.run(telemetryPacket);
        if (!cont) {
            // allow garbage collection of completed delegate so next time supplier can produce new instance
            delegate = null;
        }
        return cont;
    }

//    @Override
//    public void preview(Canvas c) {
//        if (delegate != null) delegate.preview(c);
//    }
}
