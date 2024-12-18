package org.firstinspires.ftc.teamcode.utilClass

object AsyncModifiers {
    fun awaitFor(booleanSupplier: () -> Boolean): Boolean {
        return booleanSupplier()
    }
}