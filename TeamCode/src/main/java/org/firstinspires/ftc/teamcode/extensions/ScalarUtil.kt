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

        if (c === "yellow") {
            return Scalar(255.0, 255.0, 0.0)
        } else if (c === "blue") {
            return Scalar(0.0, 0.0, 255.0)
        } else if (c === "green") {
            return Scalar(0.0, 255.0, 0.0)
        } else if (c === "red") {
            return Scalar(255.0, 0.0, 0.0)
        } else if (c === "black") {
            return Scalar(0.0, 0.0, 0.0)
        } else if (c === "white") {
            return Scalar(255.0, 255.0, 255.0)
        } else {
            return Scalar(255.0, 255.0, 255.0)
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
                when (name) {
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