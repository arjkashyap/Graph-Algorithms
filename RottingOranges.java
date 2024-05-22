// 0 = empty; 1 = fresh; 2 = rotten
class Solution {
    private final int[][] dirs = {{0,1},{1,0},{-1,0},{0,-1}};

    public int orangesRotting(int[][] grid) {
        int time = -1, rotten=0,total=0;
        int M = grid.length, N = grid[0].length;
        Queue<int[]> q = new LinkedList<>();

        for(int i = 0; i < M; i++) {
            for(int j = 0; j < N; j++) {
                if(grid[i][j] != 0)
                    total++;
                if(grid[i][j]==2) {
                    q.add(new int[]{i, j});
                    grid[i][j] = -1;
                    rotten++;
                }
            }
        }

        // perform bfs along on th queue
        while(!q.isEmpty()) {
            int size = q.size();
            for(int i = 0; i < size; i++) {
                int[] curr = q.poll();
                int r = curr[0], c = curr[1];
                for(var dir : dirs) {
                    int nr = r+dir[0], nc = c+dir[1];

                    if(isValid(grid, nr, nc)) {
                        q.add(new int[]{nr, nc});
                        grid[nr][nc]=-1;
                        rotten++;
                    }
                }

            }
            time++;
        }
        if(total-rotten > 0) return -1;
        if(time==-1) return 0;
        return time;
    }

    private boolean isValid(int[][] grid, int r, int c) {
        if(r < 0 || r >= grid.length || c < 0 || c >= grid[0].length)
            return false;
        
        return grid[r][c]==1;
    }
}
/*

Add the rotten oranges to the queue to init the stage where time is 0. Mark the visted oranges as -1 
if you're allowed to modify the grid else use a hashset or matrix. 
While q is not empty, visit the neighbours of the q front for every level and increase  the time till.
*/