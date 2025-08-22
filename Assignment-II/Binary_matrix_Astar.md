```
import java.util.*;

public class AStarPathFinding {
    static class Cell {
        int x, y;
        double g; // cost from start
        double h; // heuristic to goal
        Cell parent;

        Cell(int x, int y, double g, double h, Cell parent) {
            this.x = x;
            this.y = y;
            this.g = g;
            this.h = h;
            this.parent = parent;
        }
    }

    // 8 possible moves
    static int[][] directions = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
    };

    // Heuristic (Euclidean distance)
    static double heuristic(int x1, int y1, int x2, int y2) {
        return Math.sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    }

    // Check valid move
    static boolean isValid(int x, int y, int[][] grid, boolean[][] visited) {
        int n = grid.length;
        return x >= 0 && y >= 0 && x < n && y < n && grid[x][y] == 0 && !visited[x][y];
    }

    // Reconstruct path
    static List<int[]> constructPath(Cell end) {
        List<int[]> path = new ArrayList<>();
        while (end != null) {
            path.add(new int[]{end.x, end.y});
            end = end.parent;
        }
        Collections.reverse(path);
        return path;
    }

    // A* Search
    static List<int[]> aStarSearch(int[][] grid) {
        int n = grid.length;
        boolean[][] visited = new boolean[n][n];

        PriorityQueue<Cell> pq = new PriorityQueue<>(
            (a, b) -> Double.compare(a.g + a.h, b.g + b.h)
        );

        Cell start = new Cell(0, 0, 0, heuristic(0, 0, n - 1, n - 1), null);
        pq.add(start);

        while (!pq.isEmpty()) {
            Cell curr = pq.poll();

            if (visited[curr.x][curr.y]) continue;
            visited[curr.x][curr.y] = true;

            // reached goal
            if (curr.x == n - 1 && curr.y == n - 1) {
                return constructPath(curr);
            }

            for (int[] d : directions) {
                int nx = curr.x + d[0], ny = curr.y + d[1];
                if (isValid(nx, ny, grid, visited)) {
                    double gCost = curr.g + 1; // each step costs 1
                    pq.add(new Cell(nx, ny, gCost, heuristic(nx, ny, n - 1, n - 1), curr));
                }
            }
        }
        return Collections.emptyList(); // no path
    }

    public static void main(String[] args) {
        int[][] grid = {
            {0, 1, 0},
            {0, 0, 0},
            {1, 0, 0}
        };

        List<int[]> path = aStarSearch(grid);

        if (path.isEmpty()) {
            System.out.println("No path found.");
        } else {
            System.out.println("A* Path length = " + path.size());
            for (int[] p : path) {
                System.out.print("(" + p[0] + "," + p[1] + ") ");
            }
            System.out.println();
        }
    }
}
```
