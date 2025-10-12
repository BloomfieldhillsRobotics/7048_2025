package org.firstinspires.ftc.teamcode.subsystems; // Change to your team's package

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

/**
 * A utility class to manage HuskyLens tag detection.
 * Requires the HuskyLens to be configured with the "Tag Recognition" algorithm.
 */
public class HuskyLensTagDetector {

    private final HuskyLens huskyLens;
    public HuskyLens.Block detectedTag = null; // Public variable to store the most recently detected tag

    /**
     * Initializes the HuskyLens tag detector.
     * @param hardwareMap The robot's HardwareMap.
     * @param huskyLensName The name of the HuskyLens in the robot configuration.
     */
    public HuskyLensTagDetector(HardwareMap hardwareMap, String huskyLensName) {
        huskyLens = hardwareMap.get(HuskyLens.class, huskyLensName);

        // Use a try-catch block for the I2C command to prevent runtime crashes
        try {
            huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        } catch (Exception e) {
            // Log or report the exception if necessary
            // For example: telemetry.log().add("HuskyLens Algorithm Error: " + e.getMessage());
        }
    }

    /**
     * Scans for AprilTags and updates the detectedTag variable.
     * Call this method repeatedly in your OpMode's loop.
     */
    public void scanForTags() {
        HuskyLens.Block[] blocks = huskyLens.blocks();
        detectedTag = null; // Clear the previous detection

        if (blocks.length > 0) {
            // For simplicity, we just grab the first detected block.
            // You can add more complex logic here to filter by ID, get the largest block, etc.
            detectedTag = blocks[0];
        }
    }

    /**
     * Checks if a tag was detected in the last scan.
     * @return true if a tag was detected, false otherwise.
     */
    public boolean tagWasDetected() {
        return detectedTag != null;
    }

    /**
     * Gets the ID of the most recently detected tag.
     * @return The ID of the tag, or -1 if no tag was detected.
     */
    public int getDetectedTagId() {
        return tagWasDetected() ? detectedTag.id : -1;
    }

    /**
     * Gets the X coordinate of the most recently detected tag.
     * @return The X coordinate of the tag's center, or -1 if no tag was detected.
     */
    public int getDetectedTagX() {
        return tagWasDetected() ? detectedTag.x : -1;
    }

    /**
     * Gets the Y coordinate of the most recently detected tag.
     * @return The Y coordinate of the tag's center, or -1 if no tag was detected.
     */
    public int getDetectedTagY() {
        return tagWasDetected() ? detectedTag.y : -1;
    }

    /**
     * Gets the width of the most recently detected tag's bounding box.
     * @return The width of the tag's bounding box, or -1 if no tag was detected.
     */
    public int getDetectedTagWidth() {
        return tagWasDetected() ? detectedTag.width : -1;
    }

    /**
     * Gets the height of the most recently detected tag's bounding box.
     * @return The height of the tag's bounding box, or -1 if no tag was detected.
     */
    public int getDetectedTagHeight() {
        return tagWasDetected() ? detectedTag.height : -1;
    }

    /**
     * Gets a list of all tags detected in the last scan.
     * @return A List of HuskyLens.Block objects, potentially empty.
     */
    public List<HuskyLens.Block> getAllDetectedTags() {
        HuskyLens.Block[] blocks = huskyLens.blocks();
        List<HuskyLens.Block> tagList = new ArrayList<>();
        if (blocks.length > 0) {
            for (HuskyLens.Block block : blocks) {
                tagList.add(block);
            }
        }
        return tagList;
    }
}
