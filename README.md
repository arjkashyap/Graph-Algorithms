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


### 3. Topological Sort
Topological sorting for **Directed Acyclic Graph (DAG)** is a linear ordering of vertices such that for every directed edge u-v, vertex u comes before v in the ordering.

_Note: Topological Sorting for a graph is not possible if the graph is not a DAG._

There are two ways to perform topological Sorting

- DFS 
- Kahns Algorithm (BFS approach)

As noted above, topoligcal sorting is only possible for DAG i;e directed Acyclic Graphs. We use this property of this algoirthm to perform validation tasks on graphs such as **Cycle Detection** in a DAG.

We will demonstrate the two approaches using the following problem statement 

#### Example Question
[Leetcode-210: Course Schedule II](https://leetcode.com/problems/course-schedule-ii/)

##### DFS Approach


Perform DFS on the adj List and maintain two states of the nodes 
- visited : The nodes that are done/processed
- visiting: The nodes that we are currenty traversing 

While we want to just skip the former, we want to see if during our traversal we come across a node that is being currently processed. Soon as we encounter one such node, we break out and return an empty array.
Else we just store the order in which we traverse them in an arrayList that would form our dependency tree.

```java

class Solution {
    Set<Integer> visited = new HashSet<>();
    Set<Integer> visiting = new HashSet<>();
    List<Integer> resList = new ArrayList<>();

    public int[] findOrder(int n, int[][] pre) {
        List<List<Integer>> adj = new ArrayList<>();
        for(int i = 0; i < n; i++) {
            adj.add(new ArrayList<>());
        }
        for(int[] p : pre) {
            adj.get(p[0]).add(p[1]);
        }

        for(int i = 0; i < n; i++) {
            if(visited.contains(i))
                continue;
            if(containsCycle(adj, i))
                return new int[0];
        }

        return resList.stream().mapToInt(i -> i).toArray();
    }

    private boolean containsCycle(List<List<Integer>> adj, int curr) {

        visiting.add(curr);
        for(int nei : adj.get(curr)) {
            if(visited.contains(nei)) 
                continue;
            if(visiting.contains(nei) || containsCycle(adj, nei)) 
                return true;
        }

        visiting.remove(curr);
        visited.add(curr);
        resList.add(curr);

        return false;
    }
}

```


##### Kahn's Algorithm (BFS Approach)


Calculate the indegree of each vertex while building the adj list. Indegree is the number of incoming edges to a vertex in a DAG. Enqueue the verticies which have indegree==0 
- perform bfs and on every poll item, reduce the indegree of its neighbour
- If the neighbours indegree becomes zero: enqueue it

```java
class Solution {

    public int[] findOrder(int n, int[][] pre) {
        int[] indegree = new int[n];
        List<Integer> result = new ArrayList<>();
        Map<Integer, List<Integer>> adj = new HashMap<>();
        
        for (int i = 0; i < n; i++) 
            adj.put(i, new ArrayList<>()); 
        
        for(int[] p : pre) {
            adj.get(p[1]).add(p[0]);
            indegree[p[0]]++;
        }

        Queue<Integer> q = new LinkedList<>();
        for(int i = 0; i < n; i++) {
            if(indegree[i]==0) q.add(i);
        }

        while(!q.isEmpty()) {
            int curr = q.poll();
            result.add(curr);
            for(var nei : adj.get(curr)) {
                indegree[nei]--;
                if(indegree[nei]==0)
                    q.add(nei);
            }
        }
        
        // return empty array if cycle is detected
        for(int id : indegree) {
            if(id != 0) return new int[]{};
        }

        return result.stream().mapToInt(Integer::intValue).toArray();
    }
}

```

