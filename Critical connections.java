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
