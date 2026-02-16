/*
 * STUDENT ID: 2140111
 * COURSE: DATA STRUCTURES AND ALGORITHMS - CHANNEL A
 * 
 * ALGORITHM DESCRIPTION:
 * To optimize the search for nodes within distance 'alpha', I implemented an initial sorting 
 * of nodes based on their X-coordinate. This sorting step has a complexity of O(N log N).
 * 
 * Instead of computing the Euclidean distance for every pair of nodes (which would be O(N^2)),
 * the algorithm iterates through neighbors in the sorted list. The search is pruning as soon 
 * as the difference in X-coordinates exceeds 'alpha', since the actual Euclidean distance 
 * would strictly be greater than the X-distance.
 * 
 * This strategy effectively reduces the average complexity to O(N log N), making the 
 * neighbor search operation nearly constant time for sparse graphs relative to 'alpha'.
 */

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.PriorityQueue;

public class EMST {

    // Represents a 2D point in the plane
    static class Point {
        int originalId;
        int x, y;

        public Point(int id, int x, int y) {
            this.originalId = id;
            this.x = x;
            this.y = y;
        }
    }

    // Represents a node in the Priority Queue for Prim's Algorithm
    static class Node implements Comparable<Node> {
        int id;
        double distance;

        public Node(int id, double distance) {
            this.id = id;
            this.distance = distance;
        }

        // Compare nodes based on distance for the Min-Heap
        @Override
        public int compareTo(Node other) {
            return Double.compare(this.distance, other.distance);
        }
    }

    public static void main(String[] args) {
        if (args.length < 2) {
            return;
        }

        // Parse command line arguments
        String fileName = args[0];
        double alpha;

        try {
            alpha = Double.parseDouble(args[1]);
        } catch (NumberFormatException e) {
            return;
        }

        List<Point> inputPoints = new ArrayList<>();

        // Read input data and populate the points list
        try (BufferedReader buffer = new BufferedReader(new FileReader(fileName))) {
            String line;
            int idCounter = 0;
            while ((line = buffer.readLine()) != null) {
                // Parse format: (x,y)
                String content = line.replace("(", "").replace(")", "");
                String[] parts = content.split(",");
                int x = Integer.parseInt(parts[0].trim());
                int y = Integer.parseInt(parts[1].trim());
                inputPoints.add(new Point(idCounter++, x, y));
            }
        } catch (IOException e) {
            return;
        }

        int n = inputPoints.size();
        if (n == 0) {
            System.out.println("FAIL");
            return;
        }

        // Sort points by X-coordinate to optimize neighbor search
        List<Point> sortedPointsX = new ArrayList<>(inputPoints);
        sortedPointsX.sort((p1, p2) -> Integer.compare(p1.x, p2.x));

        // Map original IDs to their new index in the sorted array for O(1) lookup
        int[] positionMap = new int[n];
        for (int i = 0; i < n; i++) {
            positionMap[sortedPointsX.get(i).originalId] = i;
        }

        // Prim's Algorithm Data Structures
        double[] minDistance = new double[n]; // Min distance to connect each node to the MST
        int[] parent = new int[n]; // Parent node index in the MST
        boolean[] visited = new boolean[n]; // Tracks visited nodes

        // Initialize distances to infinity and parents to -1
        for (int i = 0; i < n; i++) {
            minDistance[i] = Double.MAX_VALUE;
            parent[i] = -1;
        }

        PriorityQueue<Node> pq = new PriorityQueue<>(); // Manages nodes to visit by proximity

        // -- START SEARCH (Prim's) --
        minDistance[0] = 0.0;
        pq.add(new Node(0, 0.0));

        double totalWeight = 0.0;
        int connectedNodes = 0;
        List<String> emstEdges = new ArrayList<>(); // Stores edges for output if N <= 10

        while (!pq.isEmpty()) {
            // Extract node with the smallest distance
            Node current = pq.poll();
            int uId = current.id;

            if (visited[uId])
                continue;

            visited[uId] = true;
            connectedNodes++;
            totalWeight += current.distance;

            // Store edge for printing if required (N <= 10)
            if (parent[uId] != -1) {
                if (n <= 10) {
                    Point p1 = inputPoints.get(parent[uId]);
                    Point p2 = inputPoints.get(uId);
                    emstEdges.add("(" + p1.x + "," + p1.y + ")(" + p2.x + "," + p2.y + ")");
                }
            }

            // --- NEIGHBOR SEARCH OPTIMIZATION ---
            /*
             * Retrieve position in sorted array. Check neighbors in both directions
             * (left/right).
             * Break the loop immediately if the X-distance exceeds alpha.
             */
            Point u = inputPoints.get(uId);
            int sortedIdx = positionMap[uId];

            // Check RIGHT neighbors
            for (int i = sortedIdx + 1; i < n; i++) {
                Point v = sortedPointsX.get(i);
                if (v.x - u.x > alpha)
                    break; // Optimization: Stop if X-dist > alpha

                updateNeighbor(u, v, alpha, visited, minDistance, parent, pq);
            }

            // Check LEFT neighbors
            for (int i = sortedIdx - 1; i >= 0; i--) {
                Point v = sortedPointsX.get(i);
                if (u.x - v.x > alpha)
                    break; // Optimization: Stop if X-dist > alpha

                updateNeighbor(u, v, alpha, visited, minDistance, parent, pq);
            }
        }

        // -- OUTPUT --
        if (connectedNodes < n) {
            // Graph is disjoint
            System.out.println("FAIL");
        } else {
            // Print total weight formatted to 2 decimal places
            System.out.printf(Locale.US, "%.2f%n", totalWeight);

            // Print edges if N is small
            if (n <= 10) {
                for (String edge : emstEdges) {
                    System.out.println(edge);
                }
            }
        }
    }

    private static void updateNeighbor(Point u, Point v, double alpha, boolean[] visited,
            double[] minDist, int[] parent, PriorityQueue<Node> pq) {
        int vId = v.originalId;
        if (visited[vId])
            return;

        // Euclidean distance calculation
        double dist = Math.sqrt(Math.pow(u.x - v.x, 2) + Math.pow(u.y - v.y, 2));

        // If edge is valid (<= alpha) and improved (shorter than current known
        // distance)
        if (dist <= alpha && dist < minDist[vId]) {
            minDist[vId] = dist;
            parent[vId] = u.originalId;
            pq.add(new Node(vId, dist));
        }
    }
}