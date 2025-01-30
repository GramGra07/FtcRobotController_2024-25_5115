package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import org.firstinspires.ftc.teamcode.subsystems.gameSpecific.ArmSubsystem
import org.firstinspires.ftc.teamcode.subsystems.gameSpecific.ScoringSubsystem
import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.DAVars
import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.LiftVars
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
        AUTOPick,
        HUMAN,
        auto,
        LIFT
    }

    fun lift() {
        liftFunc.runInit()
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
        private var hBasketE = DAVars.hBasketE
        private var pickupP = DAVars.pickUpP
        private var humanE = DAVars.humanE
        private var humanP = DAVars.humanP

        var daState = DAState.IDLE
        lateinit var daFunc: DAFunc
    }

    private val useConfig = true
    fun update() {
        if (useConfig) {
            collapseE = DAVars.collapseE
            collapseP = DAVars.collapseP
            hSpecimenE = DAVars.hSpecimenE
            hBasketE = DAVars.hBasketE
            pickupP = DAVars.pickUpP
            humanE = DAVars.humanE
            humanP = DAVars.humanP
        }
        daFunc.runALot()
        if (daState != DAState.LIFT) {
            firstLevelLift.restartAtBeginning()
        }
        if (daState != DAState.PICKUP) {
            pickupSM.restartAtBeginning()
        }
    }

    val autoScoreFunc = DAFunc(DAState.auto, {
        scoringSubsystem.closeClaw()
        scoringSubsystem.setPitchMed()
        scoringSubsystem.setRotateIdle()
    }, { armSubsystem.setPE(2000.0, 0.0, null) }, null, armSubsystem)

    val collapseFunc = DAFunc(DAState.COLLAPSE, {
        scoringSubsystem.closeClaw()
        scoringSubsystem.setRotateIdle()
    }, { val bool = if (!auto) false else null
        armSubsystem.setPE(collapseP, collapseE, false) }, null, armSubsystem)

    val highSpecimenFunc = DAFunc(DAState.HIGH_SPECIMEN, {
        scoringSubsystem.setRotateCenter()
    }, { armSubsystem.setPE(DAVars.hSpecimenP, hSpecimenE) }, {
        scoringSubsystem.specimenRotate((DAVars.hSpecimenP * armSubsystem.degreePerTick))
    }, armSubsystem)

    val highBasketFunc = DAFunc(DAState.HIGH_BASKET, {
        scoringSubsystem.setPitchMed()
        scoringSubsystem.setRotateCenter()
    }, { armSubsystem.setPE(DAVars.hBasketP, hBasketE, true) }, {
//        armSubsystem.pMax = 0.5
    }, armSubsystem)


    enum class PickupState {
        retract,
        pivot,
        extend,
        stop
    }

    val pickupSM = SequentialRunSM.Builder<PickupState>()
        .state(PickupState.retract)
        .onEnter(PickupState.retract) {
            armSubsystem.setExtendTarget(0.0)
            if (auto){
                armSubsystem.setPitchTarget(1500.0)
            }
            scoringSubsystem.setPitchMed()
            scoringSubsystem.openClaw()
        }
        .transition(PickupState.retract) {
            armSubsystem.isExtendAtTarget(250.0)
        }
        .state(PickupState.pivot)
        .onEnter(PickupState.pivot) {
            armSubsystem.setPitchTarget(pickupP)
        }
        .transition(PickupState.pivot) {
            armSubsystem.isPitchAtTarget(200.0)
        }
        .state(PickupState.extend)
        .onEnter(PickupState.extend) {
            armSubsystem.setExtendTarget(DAVars.pickUpE)
        }
        .transition(PickupState.extend) {
            armSubsystem.bothAtTarget()
        }
        .stopRunning(PickupState.stop)
        .build()

    val pickupFunc = DAFunc(
        DAState.PICKUP,
        {
            scoringSubsystem.setPitchMed()
            scoringSubsystem.openClaw()
            scoringSubsystem.shouldRotate = true
        }, null, {
            if (!pickupSM.isStarted) {
                pickupSM.start()
            } else {
                pickupSM.update()
            }
            scoringSubsystem.setRotateAuto()
        }, armSubsystem,pickupSM
    )

    val humanFunc = DAFunc(DAState.HUMAN, {
        scoringSubsystem.setPitchMed()
        scoringSubsystem.setRotateCenter()
        scoringSubsystem.openClaw()
    }, { armSubsystem.setPE(humanP, humanE, null) }, null, armSubsystem)

    val liftFunc = DAFunc(DAState.LIFT, {
        scoringSubsystem.setPitchLow()
        scoringSubsystem.closeClaw()
    }, null, {
        if (!firstLevelLift.isStarted) {
            firstLevelLift.start()
        } else {
            firstLevelLift.update()
        }
    }, armSubsystem)

    fun isDone(tolerance: Double): Boolean {
        return daFunc.isEnded(tolerance)
    }

    class DAFunc(
        val state: DAState,
        private val funcs: Runnable,
        private val armSubFunc: Runnable?,
        private val runConstant: Runnable?,
        private val armSubsystem: ArmSubsystem,
        private val pickupSM: SequentialRunSM<PickupState>? = null
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
        }

        fun isEnded(tolerance: Double): Boolean {
            if (daState == DAState.PICKUP){
                return (pickupSM?.isRunning?.not()?:false)
            }
            return armSubsystem.isEnded(tolerance)
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



//    fun lift() {
//        usingDA = true
//        if (!firstLevelLift.isStarted) {
//            firstLevelLift.start()
//        } else {
//            firstLevelLift.update()
//        }
//    }


    enum class AutoLift {
        extend_pivot,
        hook,
        hook1st,
        collapse,
        hook2nd,
        stop
    }

    val firstLevelLift: SequentialRunSM<AutoLift> =
        SequentialRunSM.Builder<AutoLift>()
            .state(AutoLift.extend_pivot)
            .onEnter(AutoLift.extend_pivot) {
                scoringSubsystem.setPitchLow()
                scoringSubsystem.closeClaw()
                armSubsystem.setPitchTargetDegrees(LiftVars.step1P)
                armSubsystem.setExtendTarget(LiftVars.step1E)
            }
            .transition(AutoLift.extend_pivot) {
                armSubsystem.bothAtTarget(300.0)
            }
            .state(AutoLift.hook)
            .onEnter(AutoLift.hook) {
                armSubsystem.setPitchTarget(2000.0)
            }
            .transition(AutoLift.hook) {
                armSubsystem.isPitchAtTarget(200.0)
            }
            .state(AutoLift.collapse)
            .onEnter(AutoLift.collapse) {
                armSubsystem.setExtendTarget(1200.0)
            }
            .transition(AutoLift.collapse) {
                armSubsystem.isExtendAtTarget(200.0)
            }
//            .state(AutoLift.hook1st)
//            .onEnter(AutoLift.hook1st) {
//                armSubsystem.setPE(LiftVars.step2P, LiftVars.step2E)
//            }
//            .transition(AutoLift.hook1st) {
//                armSubsystem.bothAtTarget(200.0)
//            }
            .stopRunning(AutoLift.stop)
            .build()
}