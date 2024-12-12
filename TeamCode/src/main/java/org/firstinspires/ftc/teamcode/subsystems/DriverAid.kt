package org.firstinspires.ftc.teamcode.subsystems

import org.firstinspires.ftc.teamcode.subsystems.gameSpecific.ArmSubsystem
import org.firstinspires.ftc.teamcode.subsystems.gameSpecific.ScoringSubsystem
import org.gentrifiedApps.statemachineftc.SequentialRunSM

class DriverAid(
    private val scoringSubsystem: ScoringSubsystem,
    private val armSubsystem: ArmSubsystem,
    private val localizerSubsystem: LocalizerSubsystem,
) {
    fun collapse() {
        scoringSubsystem.closeClaw()
        scoringSubsystem.setPitchHigh()
        scoringSubsystem.update()
        armSubsystem.setExtendTarget(0.0)
        armSubsystem.setPitchTarget(armSubsystem.pitchBottom)
    }


    fun lift() {
        if (!liftSequence.isStarted) {
            liftSequence.start()
        } else {
            liftSequence.update()
        }
    }

    fun getRobotAngle(): Double {
        return localizerSubsystem.poseUpdater.imuData.pitch
    }

    enum class AutoLift {
        extend_pivot,
        hook1st,
        lift1st,
        pivotBack,
        sit1st,
        extend2nd,
        hook2nd,
        lift2nd,
        pivot2nd,
        sit2nd,
        collapse,
        stop
    }

    val liftSequence = SequentialRunSM.Builder<AutoLift>()
        .state(AutoLift.extend_pivot)
        .onEnter(AutoLift.extend_pivot) {
            scoringSubsystem.setPitchLow()
            scoringSubsystem.closeClaw()
            scoringSubsystem.update()
            armSubsystem.setPitchTargetDegrees(30.0)
            armSubsystem.setExtendTarget(30.0)
        }
        .transition(AutoLift.extend_pivot) {
            armSubsystem.isPitchAtTarget() && armSubsystem.isExtendAtTarget()
        }
        .state(AutoLift.hook1st)
        .onEnter(AutoLift.hook1st) {
            armSubsystem.setPitchTargetDegrees(40.0)
        }
        .transition(AutoLift.hook1st) {
            armSubsystem.isPitchAtTarget()
        }
        .state(AutoLift.lift1st)
        .onEnter(AutoLift.lift1st) {
            armSubsystem.setExtendTarget(0.0)
        }
        .transition(AutoLift.lift1st) {
            armSubsystem.isExtendAtTarget()
        }
        .state(AutoLift.pivotBack)
        .onEnter(AutoLift.pivotBack) {
            armSubsystem.setPitchTargetDegrees(10.0)
        }
        .transition(AutoLift.pivotBack) {
            armSubsystem.isPitchAtTarget()
        }
        .state(AutoLift.sit1st)
        .onEnter(AutoLift.sit1st) {
            armSubsystem.setExtendTarget(10.0)
        }
        .transition(AutoLift.sit1st) {
            armSubsystem.isExtendAtTarget()
        }
        .state(AutoLift.extend2nd)
        .onEnter(AutoLift.extend2nd) {
            armSubsystem.setExtendTarget(30.0)
        }
        .transition(AutoLift.extend2nd) {
            armSubsystem.isExtendAtTarget()
        }
        .state(AutoLift.hook2nd)
        .onEnter(AutoLift.hook2nd) {
            armSubsystem.setPitchTargetDegrees(40.0)
        }
        .transition(AutoLift.hook2nd) {
            armSubsystem.isPitchAtTarget()
        }
        .state(AutoLift.lift2nd)
        .onEnter(AutoLift.lift2nd) {
            armSubsystem.setExtendTarget(0.0)
        }
        .transition(AutoLift.lift2nd) {
            armSubsystem.isExtendAtTarget()
        }
        .state(AutoLift.pivot2nd)
        .onEnter(AutoLift.pivot2nd) {
            armSubsystem.setPitchTargetDegrees(10.0)
        }
        .transition(AutoLift.pivot2nd) {
            armSubsystem.isPitchAtTarget()
        }
        .state(AutoLift.sit2nd)
        .onEnter(AutoLift.sit2nd) {
            armSubsystem.setExtendTarget(10.0)
        }
        .transition(AutoLift.sit2nd) {
            armSubsystem.isExtendAtTarget()
        }
        .state(AutoLift.collapse)
        .onEnter(AutoLift.collapse) {
            collapse()
        }
        .transition(AutoLift.collapse) {
            armSubsystem.isExtendAtTarget()
        }
        .stopRunning(AutoLift.stop)
        .build()

}