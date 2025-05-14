#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#define SIZE 16
#define GOAL_ROW 7
#define GOAL_COL 7
#define START_ROW 0
#define START_COL 0

// Maze representation (0 = open, 1 = wall)
int maze[SIZE][SIZE] = {
    // Add your maze layout here (1 for walls, 0 for open paths)
    {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    {0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 1},
    {1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 0, 1, 0, 1},
    {1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 1},
    {1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1},
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0},
    {1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1, 1, 1},
    {1, 0, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 0},
    {1, 0, 1, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 1, 1},
    {1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1},
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
};

// Directions: Right, Down, Left, Up
int dirRow[] = {0, 1, 0, -1};
int dirCol[] = {1, 0, -1, 0};

// Flood fill map (stores the distance to the goal)
int floodMap[SIZE][SIZE];

// Checks if a given cell is within bounds and not blocked by a wall
bool isValid(int row, int col) {
    return row >= 0 && row < SIZE && col >= 0 && col < SIZE && maze[row][col] == 0;
}

// Initializes the flood map with maximum distances
void initializeFloodMap() {
    for (int i = 0; i < SIZE; i++) {
        for (int j = 0; j < SIZE; j++) {
            floodMap[i][j] = SIZE * SIZE; // Set to a large value (infinity)
        }
    }
    floodMap[GOAL_ROW][GOAL_COL] = 0; // Set goal position to 0
}

// Performs the flood fill algorithm to calculate distances to the goal
void floodFill() {
    bool updated = true;
    while (updated) {
        updated = false;
        for (int row = 0; row < SIZE; row++) {
            for (int col = 0; col < SIZE; col++) {
                if (maze[row][col] == 0) { // Only check open cells
                    int minNeighbor = floodMap[row][col];
                    for (int i = 0; i < 4; i++) {
                        int newRow = row + dirRow[i];
                        int newCol = col + dirCol[i];
                        if (isValid(newRow, newCol)) {
                            minNeighbor = (floodMap[newRow][newCol] + 1 < minNeighbor)
                                          ? floodMap[newRow][newCol] + 1
                                          : minNeighbor;
                        }
                    }
                    if (minNeighbor < floodMap[row][col]) {
                        floodMap[row][col] = minNeighbor;
                        updated = true;
                    }
                }
            }
        }
    }
}

// Moves the mouse through the maze following the flood map
void solveMaze() {
    int currentRow = START_ROW;
    int currentCol = START_COL;
    
    printf("Path:\n(%d, %d)\n", currentRow, currentCol);
    
    while (currentRow != GOAL_ROW || currentCol != GOAL_COL) {
        int nextRow = currentRow;
        int nextCol = currentCol;
        int minValue = floodMap[currentRow][currentCol];

        for (int i = 0; i < 4; i++) {
            int newRow = currentRow + dirRow[i];
            int newCol = currentCol + dirCol[i];

            if (isValid(newRow, newCol) && floodMap[newRow][newCol] < minValue) {
                minValue = floodMap[newRow][newCol];
                nextRow = newRow;
                nextCol = newCol;
            }
        }

        currentRow = nextRow;
        currentCol = nextCol;
        printf("(%d, %d)\n", currentRow, currentCol);
    }
}

int main() {
    // Initialize flood map
    initializeFloodMap();

    // Perform flood fill
    floodFill();

    // Solve the maze by following the flood map
    solveMaze();

    return 0;
}
