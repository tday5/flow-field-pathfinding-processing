class Sector {
  private Point start;
  private int rows;
  private int cols;

  Sector(Point start, int rows, int cols) {
    this.start = start;
    this.rows = rows;
    this.cols = cols;
  }

  // Always returns a list of capacity 8, representing all 8 neighbors
  // a grid square might have. If a neighbor is missing, returns
  // null in its place
  ArrayList<Point> GetNeighbors(Point square) {
    ArrayList<Point> neighbors = new ArrayList<Point>(8);
    if (square.x + 1 < rows) {
      neighbors.add(new Point(square.x+1, square.y));
    } else {
      neighbors.add(null);
    }
    if (square.x + 1 < rows && square.y + 1 < cols) {
      neighbors.add(new Point(square.x+1, square.y+1));
    } else {
      neighbors.add(null);
    }
    if (square.y + 1 < cols) {
      neighbors.add(new Point(square.x, square.y+1));
    } else {
      neighbors.add(null);
    }
    if (square.x - 1 >= 0 && square.y + 1 < cols) {
      neighbors.add(new Point(square.x-1, square.y+1));
    } else {
      neighbors.add(null);
    }
    if (square.x - 1 >= 0) {
      neighbors.add(new Point(square.x-1, square.y));
    } else {
      neighbors.add(null);
    }
    if (square.x - 1 >= 0 && square.y - 1 >= 0) {
      neighbors.add(new Point(square.x-1, square.y-1));
    } else {
      neighbors.add(null);
    }
    if (square.y - 1 >= 0) {
      neighbors.add(new Point(square.x, square.y-1));
    } else {
      neighbors.add(null);
    }
    if (square.x + 1 < rows && square.y - 1 >= 0) {
      neighbors.add(new Point(square.x+1, square.y-1));
    } else {
      neighbors.add(null);
    }
    return neighbors;
  }

  // LOS algorithm inspired by howtorts.github.io/2014/01/30/Flow-Fields-LOS.html
  private void LineOfSight(Point p, Point a, boolean[][] losField, int[][] integrationField) {
    boolean hasLos = false;

    int xDif = p.x - a.x;
    int yDif = p.y - a.y;
    int xDifAbs = Math.abs(xDif);
    int yDifAbs = Math.abs(yDif);
    int xDifOne = Integer.signum(xDif);
    int yDifOne = Integer.signum(yDif);

    // Check diagonal if straight diagonal
    if (yDifAbs == xDifAbs && losField[a.x + xDifOne][a.y + yDifOne] &&
      integrationField[a.x + xDifOne][a.y] <= integrationField[a.x][a.y] &&
      integrationField[a.x][a.y + yDifOne] <= integrationField[a.x][a.y]) {
      hasLos = true;
    }

    //Check in the x direction if more X than Y
    if (xDifAbs > yDifAbs && losField[a.x + xDifOne][a.y] &&
      integrationField[a.x + xDifOne][a.y] <= integrationField[a.x][a.y]) {
      hasLos = true;
    }
    //Check in the y direction if more Y than x
    if (yDifAbs > xDifAbs && losField[a.x][a.y + yDifOne] &&
      integrationField[a.x][a.y + yDifOne] <= integrationField[a.x][a.y]) {
      hasLos = true;
    }

    losField[a.x][a.y] = hasLos;
  }

  private void CalcIntegrationField(Point p, int[][] integrationField, boolean[][] losField) {
    for (int i = 0; i < rows; ++i) {
      for (int j = 0; j < cols; ++j) {
        integrationField[i][j] = Integer.MAX_VALUE;
        losField[i][j] = false;
      }
    }

    Stack<Point> open = new Stack<Point>();
    integrationField[p.x][p.y] = 0;
    losField[p.x][p.y] = true;
    open.push(new Point(p.x, p.y));
    while (!open.isEmpty()) {
      Point a = open.pop();
      ArrayList<Point> neighbors = this.GetNeighbors(a);
      for (int i = 0; i < neighbors.size(); ++i) {
        if (neighbors.get(i) == null ||
          grid.costField[grid.SectorToGrid(this.start, neighbors.get(i)).x][grid.SectorToGrid(this.start, neighbors.get(i)).y] == Byte.MAX_VALUE) {
          continue;
        }
        int newCost = integrationField[a.x][a.y] +
          grid.costField[grid.SectorToGrid(this.start, neighbors.get(i)).x][grid.SectorToGrid(this.start, neighbors.get(i)).y];
        if (newCost < integrationField[neighbors.get(i).x][neighbors.get(i).y]) {
          integrationField[neighbors.get(i).x][neighbors.get(i).y] = newCost;
          open.push(neighbors.get(i));
        }
      }

      if (!a.equals(p)) {
        LineOfSight(p, a, losField, integrationField);
      }
    }
  }

  void CalcFlowField(Point p, int[][] integrationField, boolean[][] losField, PVector[][] flowField) {
    this.CalcIntegrationField(p, integrationField, losField);

    for (int i = 0; i < rows; ++i) {
      for (int j = 0; j < cols; ++j) {
        flowField[i][j] = new PVector(0, 0);
        
        if (integrationField[i][j] == Integer.MAX_VALUE || (i == p.x && j == p.y)) {
          continue;
        }

        ArrayList<Point> neighbors = this.GetNeighbors(new Point(i, j));

        // Find cheapest neighbor and aim towards it
        int cheapest = Integer.MAX_VALUE;
        Point cheapestNeighbor = null;
        for (int k = 0; k < neighbors.size(); ++k) {
          if (neighbors.get(k) == null) {
            continue;
          }
          if (integrationField[neighbors.get(k).x][neighbors.get(k).y] < cheapest) {
            // Set it to cheapest
            cheapest = integrationField[neighbors.get(k).x][neighbors.get(k).y];
            cheapestNeighbor = neighbors.get(k);
          }
        }
        if (cheapestNeighbor == null) {
          flowField[i][j] = new PVector(0, 0);
        } else {
          flowField[i][j] = new PVector(cheapestNeighbor.x - i, cheapestNeighbor.y - j);
        }
      }
    }
  }
}
