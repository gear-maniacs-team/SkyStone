package net.gearmaniacs.teamcode.utils

import android.content.Context
import com.google.gson.Gson
import com.google.gson.reflect.TypeToken
import net.gearmaniacs.teamcode.milkshake.CurvePoint

object PathParser {

    fun parseAsset(context: Context, fileName: String): List<CurvePoint> {
        var text: String? = null

        context.assets.open(fileName).bufferedReader().use {
            text = it.readText()
        }

        return parseString(text ?: throw IllegalStateException("Could not read file: $fileName"))
    }

    fun parseString(text: String): List<CurvePoint> {
        val listType = object : TypeToken<java.util.ArrayList<CurvePoint?>?>() {}.type
        return Gson().fromJson<List<CurvePoint>>(text, listType)
    }
}
