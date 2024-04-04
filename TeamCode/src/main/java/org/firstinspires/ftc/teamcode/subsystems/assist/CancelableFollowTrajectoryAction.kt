import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action

class CancelableFollowTrajectoryAction(t: Action) :
    Action {
    private var action: Action
    private var cancelled = false

    init {
        action = t
    }

    override fun run(p: TelemetryPacket): Boolean {
        if (cancelled) {
            return false
        }
        return action.run(p)
    }

    fun cancelAbruptly() {
        cancelled = true
    }
}
