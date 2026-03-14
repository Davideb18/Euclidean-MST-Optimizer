package com.emst.optimizer;

/**
 * Represents a 2D point in the plane with an original ID for tracking.
 */
public class Point {
    public final int originalId;
    public final int x;
    public final int y;

    public Point(int originalId, int x, int y) {
        this.originalId = originalId;
        this.x = x;
        this.y = y;
    }

    @Override
    public String toString() {
        return "(" + x + "," + y + ")";
    }
}
