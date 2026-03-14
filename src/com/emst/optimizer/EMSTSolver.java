package com.emst.optimizer;

import java.util.ArrayList;
import java.util.List;
import java.util.PriorityQueue;

/**
 * Core solver for the Euclidean Minimum Spanning Tree problem.
 * Uses Prim's algorithm with X-coordinate sorting pruning.
 */
public class EMSTSolver {
    private final List<Point> points;
    private final double alpha;
    private final int n;

    private double totalWeight = 0;
    private final List<Point[]> edges = new ArrayList<>();
    private boolean solved = false;
    private boolean disjoint = false;

    public EMSTSolver(List<Point> points, double alpha) {
        this.points = points;
        this.alpha = alpha;
        this.n = points.size();
    }

    public void solve() {
        if (n == 0) {
            disjoint = true;
            return;
        }

        // Sort points by X-coordinate to optimize neighbor search
        List<Point> sortedPointsX = new ArrayList<>(points);
        sortedPointsX.sort((p1, p2) -> Integer.compare(p1.x, p2.x));

        // Map original IDs to their new index in the sorted array for O(1) lookup
        int[] positionMap = new int[n];
        for (int i = 0; i < n; i++) {
            positionMap[sortedPointsX.get(i).originalId] = i;
        }

        double[] minDistance = new double[n];
        int[] parent = new int[n];
        boolean[] visited = new boolean[n];

        for (int i = 0; i < n; i++) {
            minDistance[i] = Double.MAX_VALUE;
            parent[i] = -1;
        }

        PriorityQueue<MSTNode> pq = new PriorityQueue<>();
        minDistance[0] = 0.0;
        pq.add(new MSTNode(0, 0.0));

        int connectedNodes = 0;

        while (!pq.isEmpty()) {
            MSTNode current = pq.poll();
            int uId = current.id;

            if (visited[uId]) continue;

            visited[uId] = true;
            connectedNodes++;
            totalWeight += current.distance;

            if (parent[uId] != -1 && n <= 10) {
                edges.add(new Point[]{points.get(parent[uId]), points.get(uId)});
            }

            Point u = points.get(uId);
            int sortedIdx = positionMap[uId];

            // Check RIGHT neighbors
            for (int i = sortedIdx + 1; i < n; i++) {
                Point v = sortedPointsX.get(i);
                if (v.x - u.x > alpha) break;
                updateNeighbor(u, v, minDistance, parent, pq, visited);
            }

            // Check LEFT neighbors
            for (int i = sortedIdx - 1; i >= 0; i--) {
                Point v = sortedPointsX.get(i);
                if (u.x - v.x > alpha) break;
                updateNeighbor(u, v, minDistance, parent, pq, visited);
            }
        }

        if (connectedNodes < n) {
            disjoint = true;
        }
        solved = true;
    }

    private void updateNeighbor(Point u, Point v, double[] minDist, int[] parent, PriorityQueue<MSTNode> pq, boolean[] visited) {
        int vId = v.originalId;
        if (visited[vId]) return;

        double dist = Math.sqrt(Math.pow(u.x - v.x, 2) + Math.pow(u.y - v.y, 2));

        if (dist <= alpha && dist < minDist[vId]) {
            minDist[vId] = dist;
            parent[vId] = u.originalId;
            pq.add(new MSTNode(vId, dist));
        }
    }

    public boolean isDisjoint() {
        return disjoint;
    }

    public double getTotalWeight() {
        return totalWeight;
    }

    public List<Point[]> getEdges() {
        return edges;
    }
}
