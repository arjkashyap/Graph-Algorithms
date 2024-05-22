# Graph-Algorithms
Most common graph algorithms, use cases and implementations


### 1. Depth First Search (DFS)
Depth First Traversal (or DFS) for a graph is similar to Depth First Traversal of a tree. The only catch here is, that, unlike trees, graphs may contain cycles (a node may be visited twice). 

_To avoid processing a node more than once, use a boolean visited array._

- Time complexity: O(V + E), where V is the number of vertices and E is the number of edges in the graph.

- Auxiliary Space: O(V + E), since an extra visited array of size V is required, And stack size for iterative call to DFS function.

#### Use case
- If you're required to 'walk' around a path
- Generate all paths
- Topological search (Cycle Detection)
- Can be brute force for N number of scenarios

**Can be used for both Directed or Bidirectional graphs**

#### Example Question
[Leetcode-200: Number Of Islands](https://leetcode.com/problems/number-of-islands/)


##### Approach
Go through the given grid and whenever you enounter a '1' just increase res by one and perform a simple DFS along that cell. 
**Keep in mind:** set the cell to 'x' when processing to avoid visiting it twice both during bfs as well as the main method traversal

``` java

public void dfs(char[][] grid, int r, int c) {    
    grid[r][c]='0';

    if(r-1 >= 0 && grid[r-1][c] == '1')
        dfs(grid, r-1, c);
    
    if(r + 1 < grid.length && grid[r+1][c] == '1')
        dfs(grid, r+1, c);
    
    if(c - 1 >= 0 && grid[r][c-1] == '1')
        dfs(grid, r, c-1);
    
    if(c + 1 < grid[r].length && grid[r][c+1] == '1')
        dfs(grid, r, c+1);
}

public int numIslands(char[][] grid)
{
    int count = 0;
    for(int i = 0; i < grid.length; i++){
        for(int j = 0; j < grid[i].length; j++){
            if(grid[i][j]=='1'){
                dfs(grid, i, j);
                count++;
            }
        }
    }
    return count;
}
```

### 2. Breadth First Search (BFS)
This algorithm explores all the vertices in a graph at the current depth before moving on to the vertices at the next depth level. It starts at a specified vertex and visits all its neighbors before moving on to the next level of neighbors

_To avoid processing a node more than once, use a boolean visited array._

- Time complexity: O(V + E), where V is the number of vertices and E is the number of edges in the graph.

- Auxiliary Space: O(V), BFS uses a queue to keep track of the vertices that need to be visited. In the worst case, the queue can contain all the vertices in the graph. Therefore, the space complexity of BFS is O(V), where V and E are the number of vertices and edges in the given graph.

#### Use case
- Shortest Path Finding
- Cycle Detection
- BFS can be used to identify connected components in a graph
- Any operation that requires us to go neighbour by neighbour


**Can be used for both Directed or Bidirectional graphs**

#### Example Question
[Leetcode-994: Rotting Oranges](https://leetcode.com/problems/rotting-oranges/description/)

```java
private final int[][] dirs = {{0, 1}, {1, 0}, {-1, 0}, {0, -1}};

public int orangesRotting(int[][] grid) {
    int time = -1, rotten = 0, total = 0;
    int M = grid.length, N = grid[0].length;
    Queue<int[]> q = new LinkedList<>();

    for (int i = 0; i < M; i++) {
        for (int j = 0; j < N; j++) {
            if (grid[i][j] != 0)
                total++;
            if (grid[i][j] == 2) {
                q.add(new int[]{i, j});
                grid[i][j] = -1;
                rotten++;
            }
        }
    }

    // Perform BFS on the queue
    while (!q.isEmpty()) {
        int size = q.size();
        for (int i = 0; i < size; i++) {
            int[] curr = q.poll();
            int r = curr[0], c = curr[1];
            for (var dir : dirs) {
                int nr = r + dir[0], nc = c + dir[1];

                if (isValid(grid, nr, nc)) {
                    q.add(new int[]{nr, nc});
                    grid[nr][nc] = -1;
                    rotten++;
                }
            }
        }
        time++;
    }

    if (total - rotten > 0) return -1;
    if (time == -1) return 0;
    return time;
}

private boolean isValid(int[][] grid, int r, int c) {
    if (r < 0 || r >= grid.length || c < 0 || c >= grid[0].length)
        return false;

    return grid[r][c] == 1;
}

```

##### Approach
Add the rotten oranges to the queue to init the stage where time is 0. Mark the visted oranges as -1 if you're allowed to modify the grid else use a hashset or matrix. 
While q is not empty, visit the neighbours of the q front for every level and increase  the time till.
**Keep in mind:**  Be careful with the visited nodes. 