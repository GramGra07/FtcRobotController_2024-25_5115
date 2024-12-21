package org.firstinspires.ftc.teamcode.subsystems

import org.firstinspires.ftc.teamcode.subsystems.gameSpecific.ArmSubsystem
import org.firstinspires.ftc.teamcode.subsystems.gameSpecific.ScoringSubsystem
import org.gentrifiedApps.statemachineftc.SequentialRunSM

class DriverAid(
    private val scoringSubsystem: ScoringSubsystem,
    private val armSubsystem: ArmSubsystem,
    private val localizerSubsystem: LocalizerSubsystem,
) {
    var usingDA = false

    enum class DAState {
        IDLE,
        COLLAPSE,
        HIGH_SPECIMEN,
        HIGH_BASKET,
        PICKUP,
        HUMAN
    }

    var daState = DAState.IDLE

    fun collapse() {
        daState = DAState.COLLAPSE
    }

    fun highSpecimen() {
        daState = DAState.HIGH_SPECIMEN
    }

    fun highBasket() {
        daState = DAState.HIGH_BASKET
    }

    fun pickup() {
        daState = DAState.PICKUP
    }

    fun human() {
        daState = DAState.HUMAN
    }

    fun update() {
//        if (usingDA) {
        when (daState) {
            DAState.COLLAPSE -> {
                collapseSequence(scoringSubsystem)
            }

            DAState.HIGH_SPECIMEN -> {
                highSpecimenSequence(scoringSubsystem)
            }

            DAState.HIGH_BASKET -> {
                highBasketSequence(scoringSubsystem)
            }

            DAState.PICKUP -> {
                pickupSequence(scoringSubsystem)
            }

            DAState.HUMAN -> {
                humanSequence(scoringSubsystem)
            }

            DAState.IDLE -> {

            }
        }
//        }
    }

    private fun collapseSequence(scoringSubsystem: ScoringSubsystem) {
        usingDA = true
        scoringSubsystem.closeClaw()
        scoringSubsystem.setPitchHigh()
        armSubsystem.setPE(0.0, 0.0, false)
        end()
    }

    private fun highSpecimenSequence(scoringSubsystem: ScoringSubsystem) {
        usingDA = true
        armSubsystem.setPE(1200.0, 1153.0)
        scoringSubsystem.setPitchLow()
        scoringSubsystem.setRotateCenter()
        end()
    }

    private fun highBasketSequence(scoringSubsystem: ScoringSubsystem) {
        usingDA = true
        scoringSubsystem.setPitchMed()
        scoringSubsystem.setRotateCenter()
        armSubsystem.pMax = 0.5
        armSubsystem.setPE(2000.0, 2250.0, true)
        end()
    }

    private fun pickupSequence(scoringSubsystem: ScoringSubsystem) {
        usingDA = true
        scoringSubsystem.setPitchMed()
        armSubsystem.setPE(100.0, 750.0, false)
        if (armSubsystem.isExtendAtTarget(100.0) && armSubsystem.isPitchAtTarget(100.0)) {
            scoringSubsystem.setRotateCenter()
            scoringSubsystem.setPitchLow()
            scoringSubsystem.openClaw()
        }
        end()
    }

    private fun humanSequence(scoringSubsystem: ScoringSubsystem) {
        usingDA = true
        scoringSubsystem.setPitchMed()
        scoringSubsystem.setRotateCenter()
        scoringSubsystem.openClaw()
        armSubsystem.setPE(250.0, 200.0)
        end()
    }

    private fun end(
    ) {
        if (armSubsystem.isExtendAtTarget(100.0) && armSubsystem.isPitchAtTarget(100.0)) {
            daState = DAState.IDLE
        }
    }

    private var liftState = false
    fun easyLiftL1() {
        usingDA = true
        if (!liftState) {
            scoringSubsystem.setPitchLow()
            scoringSubsystem.closeClaw()
            armSubsystem.setPE(2000.0, 1300.0)
        }
        if (armSubsystem.isExtendAtTarget(100.0) && armSubsystem.isPitchAtTarget(200.0)) {
            liftState = true
            armSubsystem.setPE(100.0, 500.0)
        }
    }

    fun lift() {
        usingDA = true
        if (!liftSequence.isStarted) {
            liftSequence.start()
        } else {
            liftSequence.update()
        }
    }

//    fun getRobotAngle(): Double {
//        return localizerSubsystem.poseUpdater.imuData.pitch
//    }

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

    private val liftSequenceBuilder: SequentialRunSM.Builder<AutoLift> =
        SequentialRunSM.Builder<AutoLift>()
            .state(AutoLift.extend_pivot)
            .onEnter(AutoLift.extend_pivot) {
                scoringSubsystem.setPitchLow()
                scoringSubsystem.closeClaw()
                armSubsystem.setPE(2000.0, 1300.0)
            }
            .transition(AutoLift.extend_pivot) {
                armSubsystem.isPitchAtTarget() && armSubsystem.isExtendAtTarget()
            }
            .state(AutoLift.hook1st)
            .onEnter(AutoLift.hook1st) {
                armSubsystem.setPE(100.0, 500.0)
            }
            .transition(AutoLift.hook1st) {
                armSubsystem.isPitchAtTarget() && armSubsystem.isExtendAtTarget()
            }
            .state(AutoLift.lift1st)
            .onEnter(AutoLift.lift1st) {
                armSubsystem.setPE(100.0, 1300.0)
            }
            .transition(AutoLift.lift1st) {
                armSubsystem.isPitchAtTarget() && armSubsystem.isExtendAtTarget()
            }
            .stopRunning(AutoLift.stop)
    private val liftSequence: SequentialRunSM<AutoLift> = liftSequenceBuilder.build()
}