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