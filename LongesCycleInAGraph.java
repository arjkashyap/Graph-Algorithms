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