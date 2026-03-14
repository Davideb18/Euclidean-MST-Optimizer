package com.emst.optimizer;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public class Main {
    public static void main(String[] args) {
        if (args.length < 2) {
            System.err.println("Usage: java com.emst.optimizer.Main <filename> <alpha>");
            return;
        }

        String fileName = args[0];
        double alpha;
        try {
            alpha = Double.parseDouble(args[1]);
        } catch (NumberFormatException e) {
            System.err.println("Invalid alpha value.");
            return;
        }

        List<Point> points = readPoints(fileName);
        if (points == null || points.isEmpty()) {
            System.out.println("FAIL");
            return;
        }

        EMSTSolver solver = new EMSTSolver(points, alpha);
        solver.solve();

        if (solver.isDisjoint()) {
            System.out.println("FAIL");
        } else {
            System.out.printf(Locale.US, "%.2f%n", solver.getTotalWeight());
            if (points.size() <= 10) {
                for (Point[] edge : solver.getEdges()) {
                    System.out.println(edge[0].toString() + edge[1].toString());
                }
            }
        }
    }

    private static List<Point> readPoints(String fileName) {
        List<Point> points = new ArrayList<>();
        try (BufferedReader reader = new BufferedReader(new FileReader(fileName))) {
            String line;
            int id = 0;
            while ((line = reader.readLine()) != null) {
                if (line.trim().isEmpty()) continue;
                String content = line.replace("(", "").replace(")", "");
                String[] parts = content.split(",");
                if (parts.length < 2) continue;
                int x = Integer.parseInt(parts[0].trim());
                int y = Integer.parseInt(parts[1].trim());
                points.add(new Point(id++, x, y));
            }
            return points;
        } catch (IOException | NumberFormatException e) {
            return null;
        }
    }
}
