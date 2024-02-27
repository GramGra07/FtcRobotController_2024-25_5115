package org.firstinspires.ftc.teamcode.extensions

import org.opencv.core.Scalar

object ScalarUtil {

    fun scalarVals(color: String): Scalar {
        var c: String = color
        when (color) {
            "red" -> c = "red"
            "blue" -> c = "blue"
            "yellow" -> c = "yellow"
            "green" -> c = "green"
            "white" -> c = "white"
        }

        return if (c === "yellow") {
            Scalar(255.0, 255.0, 0.0)
        } else if (c === "blue") {
            Scalar(0.0, 0.0, 255.0)
        } else if (c === "green") {
            Scalar(0.0, 255.0, 0.0)
        } else if (c === "red") {
            Scalar(255.0, 0.0, 0.0)
        } else if (c === "black") {
            Scalar(0.0, 0.0, 0.0)
        } else if (c === "white") {
            Scalar(255.0, 255.0, 255.0)
        } else {
            Scalar(255.0, 255.0, 255.0)
        }
    }

    // color map below
    // https://i.stack.imgur.com/gyuw4.png
    fun fetchScalar(type: String, name: String, num: Int): Scalar {
        when (type) {
            "l" -> {
                when (name) {
                    "red" -> {
                        when (num) {
                            1 -> return Scalar(0.0, 70.0, 50.0)
                            2 -> return Scalar(172.0, 70.0, 50.0)
                        }
                        return Scalar(20.0, 100.0, 100.0)
                    }

                    "yellow" -> return Scalar(20.0, 100.0, 100.0)
                    "blue" -> return Scalar(100.0, 100.0, 100.0)
                    "white" -> return Scalar(0.0, 0.0, 100.0)
                    "green" -> return Scalar(40.0, 100.0, 100.0)
                }
            }

            "h" -> when (name) {
                "red" -> {
                    when (num) {
                        1 -> return Scalar(8.0, 255.0, 255.0)
                        2 -> return Scalar(180.0, 255.0, 255.0)
                    }
                    return Scalar(30.0, 255.0, 255.0)
                }

                "yellow" -> return Scalar(30.0, 255.0, 255.0)
                "blue" -> return Scalar(140.0, 255.0, 255.0)
                "white" -> return Scalar(180.0, 255.0, 255.0)
                "green" -> return Scalar(75.0, 255.0, 255.0)
            }
        }
        return Scalar(0.0, 0.0, 0.0)
    }
}