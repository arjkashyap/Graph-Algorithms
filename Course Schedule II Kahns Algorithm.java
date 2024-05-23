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