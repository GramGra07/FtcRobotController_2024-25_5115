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
    val auto: Boolean
) {
    init {
        daFunc = DAFunc(DAState.IDLE, {}, null, null, armSubsystem)
    }

    var usingDA = false

    enum class DAState {
        IDLE,
        COLLAPSE,
        HIGH_SPECIMEN,
        HIGH_BASKET,
        PICKUP,
        HUMAN,
        auto
    }

    fun auto() {
        autoScoreFunc.runInit()
    }

    fun collapse() {
        collapseFunc.runInit()
    }

    fun highSpecimen() {
        highSpecimenFunc.runInit()
    }

    fun highBasket() {
        highBasketFunc.runInit()
    }

    fun pickup() {
        pickupFunc.runInit()
    }

    fun human() {
        humanFunc.runInit()
    }


    companion object {

        private var collapseE = DAVars.collapseE
        private var collapseP = DAVars.collapseP

        private var hSpecimenE = DAVars.hSpecimenE
        private var hSpecimenP = DAVars.hSpecimenP
        private var hBasketE = DAVars.hBasketE
        private var hBasketP = DAVars.hBasketP
        private var pickupE = DAVars.pickUpE
        private var pickupP = DAVars.pickUpP
        private var humanE = DAVars.humanE
        private var humanP = DAVars.humanP

        var daState = DAState.IDLE
        lateinit var daFunc: DAFunc
    }

    private val useConfig = false
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
        daFunc.runALot()
    }

    val autoScoreFunc = DAFunc(DAState.auto, {
        scoringSubsystem.closeClaw()
        scoringSubsystem.setPitchMed()
        scoringSubsystem.setRotateIdle()
    }, { armSubsystem.setPE(2000.0, 0.0, null) }, null, armSubsystem)

    val collapseFunc = DAFunc(DAState.COLLAPSE, {
        scoringSubsystem.closeClaw()
        scoringSubsystem.setRotateIdle()
    }, { armSubsystem.setPE(collapseP, collapseE, false) }, null, armSubsystem)

    val highSpecimenFunc = DAFunc(DAState.HIGH_SPECIMEN, {
        scoringSubsystem.setRotateCenter()
    }, { armSubsystem.setHeight(24.5, 30.0, true, true) }, {
        scoringSubsystem.specimenRotate(armSubsystem.pAngle())
    }, armSubsystem)

    val highBasketFunc = DAFunc(DAState.HIGH_BASKET, {
        scoringSubsystem.setPitchMed()
        scoringSubsystem.setRotateCenter()
    }, { armSubsystem.setPE(hBasketP, hBasketE, true) }, {
        armSubsystem.pMax = 0.5
    }, armSubsystem)

    val pickupFunc = DAFunc(
        DAState.PICKUP,
        {
            scoringSubsystem.setPitchMed()
            scoringSubsystem.openClaw()
        }, { armSubsystem.setPE(pickupP, pickupE, false) }, {
            PIDVals.pitchPIDFCo.d = 0.00025
            scoringSubsystem.setRotateAuto()
        }, armSubsystem
    )

    val humanFunc = DAFunc(DAState.HUMAN, {
        scoringSubsystem.setPitchMed()
        scoringSubsystem.setRotateCenter()
        scoringSubsystem.openClaw()
    }, { armSubsystem.setPE(humanP, humanE, null) }, null, armSubsystem)

    fun isDone(tolerance: Double): Boolean {
        return daFunc.isEnded(tolerance)
    }

    class DAFunc(
        val state: DAState,
        private val funcs: Runnable,
        private val armSubFunc: Runnable?,
        private val runConstant: Runnable?,
        private val armSubsystem: ArmSubsystem
    ) {
        fun runInit() {
            armSubsystem.resetHitPosition()
            daState = state
            funcs.run()
            daFunc = this
        }

        fun runALot() {
            runConstant?.run()
            armSubFunc?.run()
            if (isEnded(200.0)){
                daFunc = DAFunc(DAState.IDLE, {}, null, null, armSubsystem)
            }
        }

        fun isEnded(tolerance: Double): Boolean {
            return armSubsystem.bothAtTarget(tolerance) && armSubsystem.secondActionRun
        }
    }

    class DAActions(
        private val funcs: List<Runnable>,
    ) : Action {
        override fun run(packet: TelemetryPacket): Boolean {
            funcs.forEach(Runnable::run)
            return false
        }
    }

    fun daAction(funcs: List<Runnable>): Action {
        return DAActions(funcs)
    }

    fun lift() {
        usingDA = true
        if (!firstLevelLift.isStarted) {
            firstLevelLift.start()
        } else {
            firstLevelLift.update()
        }
    }


    enum class AutoLift {
        extend_pivot,
        hook1st,
        hook2nd,
        stop
    }

    val firstLevelLift: SequentialRunSM<AutoLift> =
        SequentialRunSM.Builder<AutoLift>()
            .state(AutoLift.extend_pivot)
            .onEnter(AutoLift.extend_pivot) {
                scoringSubsystem.setPitchLow()
                scoringSubsystem.closeClaw()
                armSubsystem.setPE(LiftVars.step1P, LiftVars.step1E, false)
            }
            .transition(AutoLift.extend_pivot) {
                armSubsystem.setPE(LiftVars.step1P, LiftVars.step1E, false)
                armSubsystem.bothAtTarget()
            }
            .state(AutoLift.hook1st)
            .onEnter(AutoLift.hook1st) {
                armSubsystem.setPE(LiftVars.step2P, LiftVars.step2E)
            }
            .transition(AutoLift.hook1st) {
                armSubsystem.setPE(LiftVars.step2P, LiftVars.step2E)
                armSubsystem.bothAtTarget()
            }
            .state(AutoLift.hook2nd)
            .onEnter(AutoLift.hook2nd) {
                armSubsystem.setPE(LiftVars.step3P, LiftVars.step2E, false)
            }
            .transition(AutoLift.hook2nd) {
                armSubsystem.setPE(LiftVars.step3P, LiftVars.step2E, false)
                armSubsystem.isPitchAtTarget() && armSubsystem.isExtendAtTarget()
            }
            .stopRunning(AutoLift.stop)
            .build()
}