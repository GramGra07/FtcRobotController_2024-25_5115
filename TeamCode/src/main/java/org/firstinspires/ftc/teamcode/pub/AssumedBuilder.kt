package org.firstinspires.ftc.teamcode.pub

import org.firstinspires.ftc.teamcode.pub.builderInterfaces.AssumedDetectionBuilder

class AssumedBuilder(
    override val name: String,
    private val function: () -> Unit
) : AssumedDetectionBuilder {
    override fun execute() {
        function.invoke()
    }
}