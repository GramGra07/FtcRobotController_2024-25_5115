package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import org.firstinspires.ftc.teamcode.subsystems.gameSpecific.ArmSubsystem
import org.firstinspires.ftc.teamcode.subsystems.gameSpecific.ScoringSubsystem
import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.DAVars
import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.LiftVars
import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.PIDVals
import org.gentrifiedApps.statemachineftc.SequentialRunSM

class DriverAid(
    private val scoringSubsystem: ScoringSubsystem,
    val armSubsystem: ArmSubsystem,
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

    companion object {

        private var collapseE = 0.0
        private var collapseP = 100.0
        private var hSpecimenE = 1200.0
        private var hSpecimenP = 1200.0
        private var hBasketE = 2250.0
        private var hBasketP = 2000.0
        private var pickupE = 1100.0
        var pickupP = 150.0
        private var humanE = 200.0
        private var humanP = 350.0
    }

    private var pickupOnce = 0

    private val useConfig = true
    fun update() {
        if (useConfig) {
            collapseE = DAVars.collapseE
            collapseP = DAVars.collapseP
            hSpecimenE = DAVars.hSpecimenE
            hSpecimenP = DAVars.hSpecimenP
            hBasketE = DAVars.hBasketE
            hBasketP = DAVars.hBasketP
            pickupE = DAVars.pickUpE
            pickupP = DAVars.pickUpP
            humanE = DAVars.humanE
            humanP = DAVars.humanP
        }
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
                pickupOnce = 0
            }
        }
//        }
    }

    private fun collapseSequence(scoringSubsystem: ScoringSubsystem) {
        usingDA = true
        scoringSubsystem.closeClaw()
        scoringSubsystem.setPitchHigh()
        armSubsystem.setPE(collapseP, collapseE, false)
        end()
    }

    private fun highSpecimenSequence(scoringSubsystem: ScoringSubsystem) {
        usingDA = true
//        armSubsystem.setPE(hSpecimenP, hSpecimenE)
        armSubsystem.setHeight(26.0, 27.0, true, true)
        scoringSubsystem.specimenRotate(armSubsystem.pAngle())
        scoringSubsystem.setRotateCenter()
        end()
    }

    private fun highBasketSequence(scoringSubsystem: ScoringSubsystem) {
        usingDA = true
        scoringSubsystem.setPitchMed()
        scoringSubsystem.setRotateCenter()
        armSubsystem.pMax = 0.5
        armSubsystem.setPE(hBasketP, hBasketE, true)
        if (armSubsystem.isPitchAtTarget(200.0) && armSubsystem.isExtendAtTarget(200.0)) {
            scoringSubsystem.setPitchHigh()
        }
        end()
    }

    private fun pickupSequence(scoringSubsystem: ScoringSubsystem) {
        usingDA = true

        PIDVals.pitchPIDFCo.d = 0.00025
        scoringSubsystem.setPitchMed()
        scoringSubsystem.setRotateCenter()
        scoringSubsystem.openClaw()
        armSubsystem.setPE(pickupP, pickupE, false)
        end()
    }

    private fun humanSequence(scoringSubsystem: ScoringSubsystem) {
        usingDA = true
        scoringSubsystem.setPitchMed()
        scoringSubsystem.setRotateCenter()
        scoringSubsystem.openClaw()
        armSubsystem.setPE(humanP, humanE)
        end()
    }

    private fun end(
    ) {
        if (armSubsystem.isExtendAtTarget(100.0) && armSubsystem.isPitchAtTarget(100.0)) {
            daState = DAState.IDLE
        }
    }

    class DAActions(
        val funcs: List<Runnable>,
        val armSubsystem: ArmSubsystem,
        val scoringSubsystem: ScoringSubsystem,
        val driverAid: DriverAid
    ) : Action {
        override fun run(packet: TelemetryPacket): Boolean {
            funcs.forEach(Runnable::run)
            driverAid.update()
            armSubsystem.update()
            scoringSubsystem.update()
            return !armSubsystem.bothAtTarget()
        }
    }

    fun daAction(funcs: List<Runnable>): Action {
        return DAActions(funcs, armSubsystem, scoringSubsystem, this)
    }

    private var liftState = false
    fun easyLiftL1() {
        usingDA = true
        if (!liftState) {
            scoringSubsystem.setPitchLow()
            scoringSubsystem.closeClaw()
            armSubsystem.setPE(LiftVars.step1P, LiftVars.step2E)
        }
        if (armSubsystem.isExtendAtTarget(100.0) && armSubsystem.isPitchAtTarget(200.0)) {
            liftState = true
            armSubsystem.setPE(LiftVars.step2P, LiftVars.step2E)
        }
    }

    fun lift() {
        usingDA = true
        if (!firstLevelLift.isStarted) {
            firstLevelLift.start()
        } else {
            firstLevelLift.update()
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


    val firstLevelLift: SequentialRunSM<AutoLift> =
        SequentialRunSM.Builder<AutoLift>()
            .state(AutoLift.extend_pivot)
            .onEnter(AutoLift.extend_pivot) {
                scoringSubsystem.setPitchLow()
                scoringSubsystem.closeClaw()
                armSubsystem.setPE(LiftVars.step1P, LiftVars.step1E)
            }
            .transition(AutoLift.extend_pivot) {
                armSubsystem.isPitchAtTarget(150.0) && armSubsystem.isExtendAtTarget()
            }
            .state(AutoLift.hook1st)
            .onEnter(AutoLift.hook1st) {
                armSubsystem.setPE(LiftVars.step2P, LiftVars.step2E)
            }
            .transition(AutoLift.hook1st) {
                armSubsystem.isPitchAtTarget() && armSubsystem.isExtendAtTarget()
            }
            .state(AutoLift.hook2nd)
            .onEnter(AutoLift.hook2nd) {
                armSubsystem.setPE(100.0, LiftVars.step2E, false)
            }
            .transition(AutoLift.hook2nd) {
                armSubsystem.isPitchAtTarget() && armSubsystem.isExtendAtTarget()
            }
            .stopRunning(AutoLift.stop)
            .build()
    val liftSequence: SequentialRunSM<AutoLift> =
        SequentialRunSM.Builder<AutoLift>()
            .state(AutoLift.extend_pivot)
            .onEnter(AutoLift.extend_pivot) {
                scoringSubsystem.setPitchLow()
                scoringSubsystem.closeClaw()
                armSubsystem.setPE(LiftVars.step1P, LiftVars.step1E)
            }
            .transition(AutoLift.extend_pivot) {
                armSubsystem.isPitchAtTarget(150.0) && armSubsystem.isExtendAtTarget()
            }
            .state(AutoLift.hook1st)
            .onEnter(AutoLift.hook1st) {
                armSubsystem.setPE(LiftVars.step2P, LiftVars.step2E)
            }
            .transition(AutoLift.hook1st) {
                armSubsystem.isPitchAtTarget() && armSubsystem.isExtendAtTarget()
            }
            .state(AutoLift.hook2nd)
            .onEnter(AutoLift.hook2nd) {
                armSubsystem.setPE(100.0, LiftVars.step2E, false)
            }
            .transition(AutoLift.hook2nd) {
                armSubsystem.isPitchAtTarget() && armSubsystem.isExtendAtTarget()
            }
            .state(AutoLift.lift1st)
            .onEnter(AutoLift.lift1st) {
                armSubsystem.setPE(LiftVars.step3P, LiftVars.step3E)
            }
            .transition(AutoLift.lift1st) {
                armSubsystem.isPitchAtTarget() && armSubsystem.isExtendAtTarget()
            }
            .stopRunning(AutoLift.stop)
            .build()
}