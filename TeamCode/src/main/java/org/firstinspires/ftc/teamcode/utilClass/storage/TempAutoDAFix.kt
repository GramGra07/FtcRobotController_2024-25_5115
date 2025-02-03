package org.firstinspires.ftc.teamcode.utilClass.storage

import org.firstinspires.ftc.teamcode.utilClass.varConfigurations.DAVars

class TempAutoDAFix {
    enum class DATypes {
        COLLAPSE_E,
        COLLAPSE_P,
        HIGH_SPECIMEN_E,
        HIGH_SPECIMEN_P,
        HIGH_BASKET_E,
        HIGH_BASKET_P,
        PICKUP_E,
        PICKUP_P,
        HUMAN_E,
        HUMAN_P,
    }
    private var hash: HashMap<DATypes, Double?> = hashMapOf(
        DATypes.COLLAPSE_E to null,
        DATypes.COLLAPSE_P to null,
        DATypes.HIGH_SPECIMEN_E to null,
        DATypes.HIGH_SPECIMEN_P to null,
        DATypes.HIGH_BASKET_E to null,
        DATypes.HIGH_BASKET_P to null,
        DATypes.PICKUP_E to null,
        DATypes.PICKUP_P to null,
        DATypes.HUMAN_E to null,
        DATypes.HUMAN_P to null
    )
    fun store(type: DATypes, value: Double = getVar(type)) {
        hash[type] = value
    }
    fun retrieve(type: DATypes): Double {
        return hash[type] ?: 0.0
    }
    fun set(type: DATypes, value: Double) {
        when (type) {
            DATypes.COLLAPSE_E -> DAVars.collapseE= value
            DATypes.COLLAPSE_P -> DAVars.collapseP= value
            DATypes.HIGH_SPECIMEN_E -> DAVars.hSpecimenE= value
            DATypes.HIGH_SPECIMEN_P -> DAVars.hSpecimenP= value
            DATypes.HIGH_BASKET_E -> DAVars.hBasketE= value
            DATypes.HIGH_BASKET_P -> DAVars.hBasketP= value
            DATypes.PICKUP_E -> DAVars.pickUpE= value
            DATypes.PICKUP_P -> DAVars.pickUpP= value
            DATypes.HUMAN_E -> DAVars.humanE= value
            DATypes.HUMAN_P -> DAVars.humanP= value
        }
    }
    fun getVar(type: DATypes): Double {
        return when (type) {
            DATypes.COLLAPSE_E -> DAVars.collapseE

            DATypes.COLLAPSE_P -> DAVars.collapseP
            DATypes.HIGH_SPECIMEN_E -> DAVars.hSpecimenE
            DATypes.HIGH_SPECIMEN_P -> DAVars.hSpecimenP

            DATypes.HIGH_BASKET_E -> DAVars.hBasketE
            DATypes.HIGH_BASKET_P -> DAVars.hBasketP
            DATypes.PICKUP_E -> DAVars.pickUpE
            DATypes.PICKUP_P -> DAVars.pickUpP
            DATypes.HUMAN_E -> DAVars.humanE
            DATypes.HUMAN_P -> DAVars.humanP

        }
    }
    fun resetVar(type: DATypes) {
        when (type) {
            DATypes.COLLAPSE_E -> DAVars.collapseE = retrieve(DATypes.COLLAPSE_E)
            DATypes.COLLAPSE_P -> DAVars.collapseP = retrieve(DATypes.COLLAPSE_P)
            DATypes.HIGH_SPECIMEN_E -> DAVars.hSpecimenE = retrieve(DATypes.HIGH_SPECIMEN_E)
            DATypes.HIGH_SPECIMEN_P -> DAVars.hSpecimenP = retrieve(DATypes.HIGH_SPECIMEN_P)
            DATypes.HIGH_BASKET_E -> DAVars.hBasketE = retrieve(DATypes.HIGH_BASKET_E)
            DATypes.HIGH_BASKET_P -> DAVars.hBasketP = retrieve(DATypes.HIGH_BASKET_P)
            DATypes.PICKUP_E -> DAVars.pickUpE = retrieve(DATypes.PICKUP_E)
            DATypes.PICKUP_P -> DAVars.pickUpP = retrieve(DATypes.PICKUP_P)
            DATypes.HUMAN_E -> DAVars.humanE = retrieve(DATypes.HUMAN_E)
            DATypes.HUMAN_P -> DAVars.humanP = retrieve(DATypes.HUMAN_P)
        }
    }
        fun resetAll() {
            for (type in DATypes.values()) {
                if (hash[type] != null) {
                    resetVar(type)
                }
            }
        }

}