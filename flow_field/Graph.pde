class Pt {
  int name;  // index in graph array
  int x;
  int y;

  Pt(int name, int x, int y) {
    this.name = name;
    this.x = x;
    this.y = y;
  }
}

class PtList {
  Pt p;
  PtList next;

  PtList(Pt p, PtList next) {
    this.p = p;
    this.next = next;
  }
}

class AStarNode implements Comparable<AStarNode> {
  Pt pt;
  AStarNode parent;
  float costSoFar;
  float hCost;

  AStarNode(Pt pt, AStarNode parent, float costSoFar, float hCost) {
    this.pt = pt;
    this.parent = parent;
    this.costSoFar = costSoFar;
    this.hCost = hCost;
  }

  public int compareTo(AStarNode other) {
    if ((this.hCost + this.costSoFar) > (other.hCost + other.costSoFar)) {
      return 1;
    } else if ((this.hCost + this.costSoFar) < (other.hCost + other.costSoFar)) {
      return -1;
    } else {
      return 0;
    }
  }
}

class Graph {
  ArrayList<Pt> pts;
  ArrayList<PtList> adjList;  // Look up by "name" (index) of point
  int nextOpen;  // index of next point in array


  Graph() {
    pts = new ArrayList<Pt>();
    adjList = new ArrayList<PtList>();
    nextOpen = 0;
  }

  Pt addPt(int x, int y) {
    Pt newPt = new Pt(nextOpen++, x, y);
    pts.add(newPt);
    adjList.add(null);
    return newPt;
  }

  Pt closestPoint(int x, int y) {
    Pt closest = pts.get(0);
    float closestDist = dist(pts.get(0).x, pts.get(0).y, x, y);
    for (int i = 1; i < nextOpen; ++i) {
      if (dist(pts.get(i).x, pts.get(i).y, x, y) < closestDist) {
        closest = pts.get(i);
        closestDist = dist(pts.get(i).x, pts.get(i).y, x, y);
      }
    }
    return closest;
  }

  // assume edges are undirected
  void addEdge(int name1, int name2) {
    // Push onto existing list
    PtList newEntry = new PtList(pts.get(name2), adjList.get(name1));
    adjList.set(name1, newEntry);
    // And for the other direction, because undirected
    PtList newEntry2 = new PtList(pts.get(name1), adjList.get(name2));
    adjList.set(name2, newEntry2);
  }

  void addUndirectedEdge(int name1, int name2) {
    // Push onto existing list
    PtList newEntry = new PtList(pts.get(name2), adjList.get(name1));
    adjList.set(name1, newEntry);
  }

  // Finds the closest graph nodes and runs A* with those points
  ArrayList<Pt> aStar(int aX, int aY, int bX, int bY) {
    if (pts.size() == 0) {
      ArrayList<Pt> temp = new ArrayList<Pt>();
      temp.add(new Pt(0, bX, bY));
      return temp;
    }
    Pt a = closestPoint(aX, aY);
    Pt b = closestPoint(bX, bY);

    if (a == null || b == null) {
      return new ArrayList<Pt>();
    }
    return aStar(a, b);
  }

  ArrayList<Pt> aStar(Pt a, Pt b) {
    PriorityQueue<AStarNode> toExplore = new PriorityQueue<AStarNode>();
    toExplore.add(new AStarNode(a, null, 0, 0));

    // Init distances
    float[] distances = new float[nextOpen];
    for (int i = 0; i < nextOpen; ++i) {
      distances[i] = -1;
    }

    while (!toExplore.isEmpty()) {
      AStarNode head = toExplore.poll();
      if (head.pt.name == b.name) {
        ArrayList<Pt> solPts = new ArrayList<Pt>();
        solPts.add(head.pt);
        while (head.parent != null) {
          head = head.parent;
          solPts.add(head.pt);
        }
        return solPts;
      }

      if (distances[head.pt.name] != -1 && head.costSoFar >= distances[head.pt.name]) {
        continue;
      }
      distances[head.pt.name] = head.costSoFar;

      PtList adjs = adjList.get(head.pt.name);
      if (adjs != null) {
        toExplore.add(new AStarNode(adjs.p, head, head.costSoFar + 1, dist(adjs.p.x, adjs.p.y, b.x, b.y)));
        while (adjs.next != null) {
          adjs = adjs.next;
          toExplore.add(new AStarNode(adjs.p, head, head.costSoFar + 1, dist(adjs.p.x, adjs.p.y, b.x, b.y)));
        }
      }
    }
    return new ArrayList<Pt>();
  }

  void draw() {
    stroke(255, 0, 0);
    for (int i = 0; i < nextOpen; i++) {
      PVector a = grid.CalcScreenCoordsFromPoint(new Point(pts.get(i).x, pts.get(i).y));
      point(a.x, a.y);
    }
    stroke(0, 0, 0);
    for (int i = 0; i < nextOpen; i++) {
      PtList edge = adjList.get(i);
      while (edge != null) {
        PVector a = grid.CalcScreenCoordsFromPoint(new Point(pts.get(i).x, pts.get(i).y));
        PVector b = grid.CalcScreenCoordsFromPoint(new Point(edge.p.x, edge.p.y));
        line(a.x, a.y, b.x, b.y);
        edge = edge.next;
      }
    }
  }
}
