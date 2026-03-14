package com.emst.optimizer;

/**
 * Represents a node in the Priority Queue for Prim's Algorithm.
 */
public class MSTNode implements Comparable<MSTNode> {
    public final int id;
    public final double distance;

    public MSTNode(int id, double distance) {
        this.id = id;
        this.distance = distance;
    }

    @Override
    public int compareTo(MSTNode other) {
        return Double.compare(this.distance, other.distance);
    }
}
