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