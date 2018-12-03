package org.firstinspires.ftc.teamcode.Subsystems;

import java.util.ArrayList;
import java.util.Arrays;

public class Navigation {
    double x, y, hdg;
    int numNodes = 26;
    double[][] map = {
            {-60, -60},
            {-36, -60},
            {0, -60},
            {12, -60},
            {0, -48},
            {-60, -36},
            {-24, -24},
            {24, -24},
            {-12, -12},
            {12, -12},
            {60, -12},
            {-60, 0},
            {-48, 0},
            {0, 0},
            {48, 0},
            {60, 0},
            {-60, 12},
            {-12, 12},
            {12, 12},
            {-24, 24},
            {24, 24},
            {60, 36},
            {0, 48},
            {-12, 60},
            {0, 60},
            {36, 60},
            {60, 60}
    };

    int[][] connections =
    {
            {1, 5},
            {0, 2, 5},
            {1, 3, 4},
            {2},
            {2, 6, 7},
            {0, 1, 11},
            {4, 8, 12},
            {4, 9, 14},
            {13, 6},
            {13, 7},
            {15},
            {5, 12, 16},
            {6, 11, 13, 19},
            {8, 9, 17, 18},
            {7, 13, 15, 20},
            {10, 14, 21},
            {11},
            {13, 19},
            {13, 20},
            {15, 26},
            {19, 20, 24},
            {24},
            {22, 23, 25},
            {21, 24, 26},
            {21, 25}
    };

    public Navigation() {

    }

    //gets best path between nodes, backwards
    public ArrayList<Integer> aStar(int src, int dst) {
        ArrayList<Integer> closedSet = new ArrayList<>();
        ArrayList<Integer> openSet = new ArrayList<Integer>();
        openSet.add(src);
        double[] gScore = new double[numNodes];
        Arrays.fill(gScore, Double.MAX_VALUE);
        gScore[src] = 0;
        double[] fScore = new double[numNodes];
        Arrays.fill(fScore, Double.MAX_VALUE);
        fScore[src] = distance(src, dst);

        ArrayList<Integer> cameFrom = new ArrayList<>();
        while (openSet.size() > 0) {
            double min = Double.MAX_VALUE;
            int current = -1;
            for(int node : openSet) {
                if(fScore[node] < min) {
                    current = node;
                    min = fScore[node];
                }
            }
            if(current == dst) {
                return reconstructPath(cameFrom, current);
            }
            openSet.remove(Integer.valueOf(current));
            closedSet.add(current);
            for(int neighbor : connections[current]) {
                if(!closedSet.contains(neighbor)) {
                    double tentative_gScore = gScore[current] + distance(current, neighbor);

                    if(!openSet.contains(neighbor)) {
                        openSet.add(neighbor);
                    } else if(tentative_gScore >= gScore[neighbor]) {
                        continue;
                    }

                    cameFrom.set(neighbor, current);
                    gScore[neighbor] = tentative_gScore;
                    fScore[neighbor] = gScore[neighbor] + distance(neighbor, dst);
                }
            }
        }
        return new ArrayList<>();
    }

    private ArrayList<Integer> reconstructPath(ArrayList<Integer> cameFrom, int current) {
        ArrayList<Integer> totalPath = new ArrayList();
        totalPath.add(current);

        while (cameFrom.get(current) != null) {
            current = cameFrom.get(current);
            totalPath.add(current);
        }
        return totalPath;
    }

    public int closestNode(double x, double y) {
        double min = Double.MAX_VALUE;
        int closest = -1;
        for(int i = 0; i < numNodes; i++) {
            if(Math.hypot(x - map[i][0], y - map[i][1]) < min) {
                closest = i;
            }
        }
        return closest;
    }

    private double distance(int src, int dst) {
        return Math.hypot(map[dst][0] - map[src][0], map[dst][1] - map[src][1]);
    }
}
