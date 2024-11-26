package org.firstinspires.ftc.teamcode.subsystems

import org.firstinspires.ftc.teamcode.subsystems.gameSpecific.ArmSubsystem
import org.firstinspires.ftc.teamcode.subsystems.gameSpecific.ScoringSubsystem
import org.gentrifiedApps.statemachineftc.ParallelRunSM

object DriverAid {
    fun initAllSM(scoringSubsystem: ScoringSubsystem, armSubsystem: ArmSubsystem) {
        initCollapseSM(scoringSubsystem, armSubsystem)
    }

    enum class CollapseSMState {
        moveClaw,
        moveArm,
        moveScoring,
        stop
    }

    lateinit var collapseSM: ParallelRunSM<CollapseSMState>
    private fun initCollapseSM(scoringSubsystem: ScoringSubsystem, armSubsystem: ArmSubsystem) {
        collapseSM = ParallelRunSM.Builder<CollapseSMState>()
            .state(CollapseSMState.moveClaw)
            .onEnter(
                CollapseSMState.moveClaw
            ) {
                scoringSubsystem.closeClaw()
                scoringSubsystem.update()
            }
            .state(CollapseSMState.moveArm)
            .onEnter(
                CollapseSMState.moveArm
            ) {
                armSubsystem.autoExtend(0.0)
            }
            .state(CollapseSMState.moveScoring)
            .onEnter(
                CollapseSMState.moveScoring
            ) {
                scoringSubsystem.setPitchHigh()
                scoringSubsystem.update()
            }
            .stopRunning(
                CollapseSMState.stop
            ) { armSubsystem.extendMotor.currentPosition.toDouble() < 100 }
            .build(false, 0)
    }
}