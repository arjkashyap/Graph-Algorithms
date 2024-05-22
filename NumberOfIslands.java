class Solution {
    public int numIslands(char[][] grid) {
       int res = 0;
        for(int i = 0; i < grid.length; i++) {
            for(int j = 0; j < grid[0].length; j++){
                if(grid[i][j] == '1') {
                    bfs(grid, i, j);
                    res++;
                }
            }
        }
       return res; 
    }

    private void bfs(char[][] grid, int r, int c) {
        int[] rowCol = {r, c};
        Queue<int[]> q = new LinkedList<>();
        q.add(rowCol);

        while(!q.isEmpty()) {

            int[] curr = q.poll();
            int currR = curr[0];
            int currC = curr[1];
            grid[currR][currC] = '0';

            // Set adjacent grid as visited right after adding to queue
            // for some reason it is giving TLE if I don't do it
            if(currR + 1 < grid.length && grid[currR+1][currC]=='1') {
                q.add(new int[]{currR + 1, currC});
                grid[currR+1][currC]='0';
            }
            if(currR-1 >= 0 && grid[currR-1][currC]=='1') {
                q.add(new int[]{currR-1, currC});
                grid[currR-1][currC]='0';

            }
            if(currC+1 < grid[0].length && grid[currR][currC+1]=='1') {
                q.add(new int[]{currR, currC+1});
                grid[currR][currC+1]='0';

            }
            if(currC-1 >= 0 && grid[currR][currC-1]=='1') {
                q.add(new int[]{currR, currC-1});
                grid[currR][currC-1]='0';
            }  
        }
    }
}

/*

    i/p:
        ["1","1","1","1","0"]
        ["1","1","0","1","0"]
        ["1","1","0","0","0"]
        ["0","0","0","0","0"]
    o/p: 1


    i/p:    ["1","1","0","0","0"],
            ["1","1","0","0","0"],
            ["0","0","1","0","0"],
            ["0","0","0","1","1"]

    o/p: 3

    i/p: 
    [       ["1","1","1"],
            ["0","1","0"],
            ["1","1","1"]]

            [["x","x","x"],
            ["0","x","0"],
            ["1","x","x"]]

    Approach
        Go through the given grid and whenever you enounter a '1' just increase res
        by one and perform a simple BFS along that cell 

    KEEP IN MIND
        set the cell to 'x' when processing to avoid visiting it twice both during bfs
        as well as the main method traversal
*/