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
<br/>

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
<br/>


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
<br/>

### 4. Dijikstra's Algorithm for Shortest Path in Weighted graph
This algorithm is used for getting the shortest distance in a weighted graph that may or may not contain cycle

**Use Case:** 
- Dijkstra's algorithm solves the single-source shortest-paths problem in edge-weighted digraphs with nonnegative weights.
-  Generate a SPT (shortest path tree) with a given source as a root

The implementation is very simple, we use a priority queue to store the sum of weight from current u to that node and pick up nodes with shorter distance in greedy manner

- one set contains vertices included in the shortest-path tree, 
- other set includes vertices not yet included in the shortest-path tree. 

At every step of the algorithm, find a vertex that is in the other set (set not yet included) and has a minimum distance from the source.

Time Complexity: O(V2)
Auxiliary Space: O(V)


#### Example Question
[Leetcode-743: Network Delay Time](https://leetcode.com/problems/network-delay-time/)

```java
class Solution {
    public int networkDelayTime(int[][] times, int n, int k) {
        Map<Integer, List<int[]>> adj = new HashMap<>();
        Map<Integer, Integer> shortest = new HashMap<>();
        
        for(int i = 1; i <= n; i++) {
            adj.put(i, new ArrayList<>());
        }
        for(int[] t : times) {
            adj.get(t[0]).add(new int[]{t[1], t[2]});
        }

        PriorityQueue<int[]> pq = new PriorityQueue<>((a, b) -> Integer.compare(a[1], b[1]));
        pq.add(new int[]{k, 0});

        while(!pq.isEmpty()) {
            int[] uw = pq.poll();
            int u = uw[0], w = uw[1];

            if(shortest.containsKey(u))
                continue;
            
            shortest.put(u, w);
            System.out.println(u);
            // visist neighbours
            for(int[] vw : adj.get(u)) {
                int v = vw[0], w2 = vw[1];
                if(!shortest.containsKey(v))
                    pq.add(new int[]{v, (w + w2)});
            }
        }

        int maxTime = -1;
        for(int i = 1; i <= n; i++) {
            if(!shortest.containsKey(i))
                return -1;
        }

        for(int node : shortest.keySet()) {
            int time = shortest.get(node);
            if(time > maxTime)
                maxTime = time;
        }

        return maxTime;
    }
}
```
<br/>

### Kosaraju's Algorithm

**What is a Strongly Connected Component (SCC)?**
A strongly connected component in a **Directed Graph** is the component of a  graph that has a path from every vertex to every other vertex in that component. It can only be used in a directed graph.

![SCC in graph](https://media.geeksforgeeks.org/wp-content/uploads/20230801122248/scc_fianldrawio.png "SCC")


#### Approach
- Order the vertices in decreasing order of finish time in DFS
- Reverse all edges
- Do DFS of the reversed graph in the order obtained in step1
- For every vertex print all reachable vertex as SCC

#### Why it works ? 

Kosaraju's algorithm works because it uses two passes of depth-first search (DFS):

- First DFS Pass: This pass records the finishing times of nodes. Nodes that finish last are those that have no further nodes to visit, meaning they are deeply nested in the graph's structure.

- Transpose the Graph: Reversing all edges in the graph means that all paths are reversed, making the strongly connected components (SCCs) intact but reversing the direction of travel within them.

- Second DFS Pass: Processing nodes in the order of their finishing times (from the stack) ensures that when we start a DFS in the transposed graph, we only reach nodes within the same SCC. This is because nodes that are reachable in the original graph remain reachable in the reversed paths, effectively isolating SCCs.

By leveraging the finishing times and the transposed graph, the algorithm efficiently identifies all SCCs.

**Time complexity:** O(V * (V + M)), because for each pair of vertices we are checking whether a path exists between them.
**Auxiliary Space:** O(V) 

#### Example Question
[Leetcode-2360: Longest Cycle in a Graph](https://leetcode.com/problems/longest-cycle-in-a-graph/)

```java
class Solution {
    int count = 0;
    public int longestCycle(int[] edges) {
        // Implemting Kosarajus algirthm
        int res = 0;
        int n = edges.length;
        boolean[] vis = new boolean[n];
        Stack<Integer> st = new Stack<>();      // st used to store time taken in reverse
        Map<Integer, List<Integer>> revAdj = new HashMap<>();

        for(int i = 0; i < n; i++)
            revAdj.put(i, new ArrayList<>());

        for(int i = 0; i < n; i++) {
            if(edges[i] != -1)
                revAdj.get(edges[i]).add(i);
        }


        // begin dfs and
        for(int i = 0; i < n; i++) {
            if(vis[i]) continue;
            dfs(edges, vis, st, i);
        }
        Arrays.fill(vis, false);

        // perform dfs based on reverse order of time taken
        while(!st.isEmpty()) {
            int node = st.pop();
            if(vis[node]) continue;
            count=0;
            res = Math.max(res, dfs2(revAdj, vis, node));
        }

        return (res == 1) ? -1 : res;
    }

    private void dfs(int[] edges, boolean[] vis, Stack<Integer> st, int n) {
        vis[n]=true;
        if(edges[n] != -1 && !vis[edges[n]])
            dfs(edges, vis, st, edges[n]);
        st.push(n);
    }

    private int dfs2(Map<Integer, List<Integer>> adj, boolean[] vis, int n) {
        vis[n]=true;
        count++;
        for(int nei : adj.get(n)) {
            if(!vis[nei])
                dfs2(adj, vis, nei);
        }
        
        return count;
    }
}
```
<br/>

### Tarjan's Algorithm
Tarjan's Algorithm can also be used to find SCC in Directed Graphs. 
How is it different from Kosaraju's algoirthm
The former requires two (if not more) traversals of the graph. 

#### Example Question
[Leetcode-1192:  Critical Connections in a network](https://leetcode.com/problems/critical-connections-in-a-network/)


```java
class Solution {
    int time = 0;
    private int[] tin;
    private int[] low;
    private int[] vis;
    private final List<List<Integer>> bridges = new ArrayList<>();

    public List<List<Integer>> criticalConnections(int n, List<List<Integer>> conn) {
        tin = new int[n];
        low = new int[n];
        vis = new int[n];

        Map<Integer, List<Integer>> adj = new HashMap<>();
        for(int i = 0; i < n; i++) 
            adj.put(i, new ArrayList<>());
        
        for(var e : conn) {
            adj.get(e.get(0)).add(e.get(1));
            adj.get(e.get(1)).add(e.get(0));
        }
        
        dfs(adj, 0, -1);
        return bridges;
    }

    private void dfs(Map<Integer, List<Integer>> adj, int node, int parent) {
        
        vis[node]=1;
        tin[node]=time;
        low[node]=time;
        time++;

        for(int nei : adj.get(node)) {
            if(nei == parent) continue;

            if(vis[nei]==0) {
                dfs(adj, nei, node);
                low[node] = Math.min(low[node], low[nei]);
                // check if its a bridge
                if(tin[node] < low[nei]) 
                    bridges.add(Arrays.asList(new Integer[]{nei, node}));
            } else {
                low[node] = Math.min(low[node], low[nei]);
            }
        }
    }
}

```
<br/>


### Tarjan's Algorithm
Tarjan's Algorithm can also be used to find SCC in Directed Graphs. 
How is it different from Kosaraju's algoirthm
The former requires two (if not more) traversals of the graph. 

#### Example Question
[Leetcode-1192:  Critical Connections in a network](https://leetcode.com/problems/critical-connections-in-a-network/)

```java
class Solution {
    int count = 0;
    public int longestCycle(int[] edges) {
        // Implemting Kosarajus algirthm
        int res = 0;
        int n = edges.length;
        boolean[] vis = new boolean[n];
        Stack<Integer> st = new Stack<>();      // st used to store time taken in reverse
        Map<Integer, List<Integer>> revAdj = new HashMap<>();

        for(int i = 0; i < n; i++)
            revAdj.put(i, new ArrayList<>());

        for(int i = 0; i < n; i++) {
            if(edges[i] != -1)
                revAdj.get(edges[i]).add(i);
        }


        // begin dfs and
        for(int i = 0; i < n; i++) {
            if(vis[i]) continue;
            dfs(edges, vis, st, i);
        }
        Arrays.fill(vis, false);

        // perform dfs based on reverse order of time taken
        while(!st.isEmpty()) {
            int node = st.pop();
            if(vis[node]) continue;
            count=0;
            res = Math.max(res, dfs2(revAdj, vis, node));
        }

        return (res == 1) ? -1 : res;
    }

    private void dfs(int[] edges, boolean[] vis, Stack<Integer> st, int n) {
        vis[n]=true;
        if(edges[n] != -1 && !vis[edges[n]])
            dfs(edges, vis, st, edges[n]);
        st.push(n);
    }

    private int dfs2(Map<Integer, List<Integer>> adj, boolean[] vis, int n) {
        vis[n]=true;
        count++;
        for(int nei : adj.get(n)) {
            if(!vis[nei])
                dfs2(adj, vis, nei);
        }
        
        return count;
    }
}
```
<br/>

### Prim's Algorithm

Prim's algorithm is used to find the Minimum Spanning tree in a graph. A MST is basically a tree derived from a graph where each node is able to visit all the other nodes and there is NO cycle.

The implementation of Prim's algorithm is very close to Dijikstra's algorithm with little to no changes involved.

#### Example Question
[Leetcode-1584:  Min Cost to Connect All Points](https://leetcode.com/problems/min-cost-to-connect-all-points/)
```java

class Solution {
    public int minCostConnectPoints(int[][] points) {
        // The goal is to simply find the minimum spanning tree here 

        int res = 0;
        int n = points.length;
        Map<Integer, List<int[]>> adj = new HashMap<>();        // node1 -> {{dist, node2}}

        for(int i = 0; i < n; i++) {
            adj.put(i, new ArrayList<>());
        }
        
        for(int i = 0; i < n; i++) {
            int x1 = points[i][0], y1 = points[i][1];
            for(int j = i+1; j < n; j++) {
                int x2 = points[j][0], y2 = points[j][1];
                int d = Math.abs(x1-x2)+Math.abs(y1-y2);

                adj.get(i).add(new int[]{d, j});
                adj.get(j).add(new int[]{d, i});
            }
        }


        // Prims algorithm to find MSP
        Set<Integer> vis = new HashSet<>();
        Queue<int[]> pq = new PriorityQueue<>((a,b) -> Integer.compare(a[0],b[0]));

        pq.offer(new int[]{0,0});
        while(vis.size() < n && !pq.isEmpty()) {
            var curr = pq.poll();
            int cost = curr[0], node = curr[1];
            if(vis.contains(node)) continue;

            res += cost;
            vis.add(node);

            for(int[] nei : adj.get(node)) {
                int c = nei[0], x = nei[1];
                if(!vis.contains(x)) {
                    pq.offer(new int[]{c, x});
                }
            }

        }

        return res;
    }
}
```