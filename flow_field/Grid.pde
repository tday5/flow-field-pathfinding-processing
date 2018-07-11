class Point {
  public int x;
  public int y;

  public Point(int x, int y) {
    this.x = x;
    this.y = y;
  }

  public Point() {
    this(0, 0);
  }

  public boolean equals(Point other) {
    return x == other.x && y == other.y;
  }
}

class SectorPath {
  int[][] integrationField;
  boolean[][] losField;
  PVector[][] flowField;

  Point sectorDest;

  SectorPath(int[][] integrationField, boolean[][] losField, PVector[][] flowField, Point sectorDest) {
    this.integrationField = integrationField;
    this.losField = losField;
    this.flowField = flowField;
    this.sectorDest = sectorDest;
  }
}

class GridPath {
  SectorPath[][] sectorPaths;

  GridPath(SectorPath[][] sectorPaths) {
    this.sectorPaths = sectorPaths;
  }
}

class Grid {
  int rows;
  int cols;
  int sectorSize;

  float spaceSizeX;
  float spaceSizeY;

  private Graph portalGraph;
  private Sector[][] sectors;
  // First two levels are sector, second two are destination
  // within sector
  private SectorPath[][][][] generatedPaths;

  private byte[][] costField;
  private ArrayList<Body> gridBodies;

  Grid(int rows, int cols, int sectorSize) {
    this.rows = rows;
    this.cols = cols;

    this.spaceSizeX = (float)RES_X / (float)this.rows;
    this.spaceSizeY = (float)RES_Y / (float)this.cols;

    this.costField = new byte[rows][cols];
    for (int i = 0; i < costField.length; ++i) {
      for (int j = 0; j < costField[i].length; ++j) {
        costField[i][j] = 1;
      }
    }
    this.gridBodies = new ArrayList<Body>();
    this.PrepSectors(sectorSize);
  }

  void PrepSectors(int sectorSize) {
    if (rows % sectorSize != 0 || cols % sectorSize != 0) {
      throw new IllegalArgumentException("Bad sector size");
    }
    this.sectorSize = sectorSize;
    this.sectors = new Sector[rows / sectorSize][cols / sectorSize];
    for (int i = 0; i < this.sectors.length; ++i) {
      for (int j = 0; j < this.sectors[i].length; ++j) {
        this.sectors[i][j] = new Sector(new Point(i * sectorSize, j * sectorSize), sectorSize, sectorSize);
      }
    }
    this.CalcPortalGraph();
  }

  void CalcPortalGraph() {
    // Cleanup
    for (int i = 0; i < gridBodies.size(); ++i) {
      box2d.world.destroyBody(gridBodies.get(i));
    }
    gridBodies.clear();
    this.generatedPaths = new SectorPath[rows / sectorSize][cols / sectorSize][sectorSize][sectorSize];
    this.portalGraph = new Graph();

    for (int i = 0; i < rows; ++i) {
      for (int j = 0; j < cols; ++j) {
        // Create a hard barrier for impassable terrain
        if (costField[i][j] == Byte.MAX_VALUE) {
          // Define a polygon
          PolygonShape sd = new PolygonShape();
          sd.setAsBox(box2d.scalarPixelsToWorld(spaceSizeX / 2.0), 
            box2d.scalarPixelsToWorld(spaceSizeY / 2.0));

          // Define a fixture
          FixtureDef fd = new FixtureDef();
          fd.shape = sd;
          // Parameters that affect physics
          fd.density = 1;
          fd.friction = 0.5;
          fd.restitution = 0.2;

          // Define the body and make it from the shape
          BodyDef bd = new BodyDef();
          bd.type = BodyType.STATIC;
          bd.position.set(box2d.coordPixelsToWorld(
            new PVector(CalcScreenCoordsFromPoint(new Point(i, j)).x + (spaceSizeX / 2.0), 
            CalcScreenCoordsFromPoint(new Point(i, j)).y + (spaceSizeY / 2.0)
            )));

          Body body = box2d.createBody(bd);
          body.createFixture(fd);
          gridBodies.add(body);
        }
      }
    }

    // Add nodes that have adjacent nodes in other sectors on the right and bottom,
    // so as to not have any repeat
    ArrayList<ArrayList<ArrayList<Pt>>> sectorPts = new ArrayList<ArrayList<ArrayList<Pt>>>();
    for (int i = 0; i < sectors.length; ++i) {
      sectorPts.add(new ArrayList<ArrayList<Pt>>());
      for (int j = 0; j < sectors[i].length; ++j) {
        sectorPts.get(i).add(new ArrayList<Pt>());
      }
    }
    // For each sector, starting with the top left and ending with the bottom right,
    // check the right and bottom sides to see if there exists a "portal", two adjacent
    // clear spaces between sectors, and add an edge if so
    for (int i = 0; i < sectors.length; ++i) {
      for (int j = 0; j < sectors[i].length; ++j) {
        if (i != sectors.length - 1) {
          for (int k = sectors[i][j].start.y; k < sectors[i][j].start.y + sectors[i][j].cols; ++k) {
            if (costField[sectors[i][j].start.x + sectors[i][j].rows - 1][k] != Byte.MAX_VALUE &&
              costField[sectors[i][j].start.x + sectors[i][j].rows][k] != Byte.MAX_VALUE) {

              sectorPts.get(i).get(j).add(portalGraph.addPt(sectors[i][j].start.x + sectors[i][j].rows - 1, k));
              sectorPts.get(i+1).get(j).add(portalGraph.addPt(sectors[i][j].start.x + sectors[i][j].rows, k));

              portalGraph.addEdge(sectorPts.get(i).get(j).get(sectorPts.get(i).get(j).size()-1).name, 
                sectorPts.get(i+1).get(j).get(sectorPts.get(i+1).get(j).size()-1).name);
            }
          }
        }
        if (j != sectors[i].length - 1) {
          for (int k = sectors[i][j].start.x; k < sectors[i][j].start.x + sectors[i][j].rows; ++k) {
            if (costField[k][sectors[i][j].start.y + sectors[i][j].cols - 1] != Byte.MAX_VALUE &&
              costField[k][sectors[i][j].start.y + sectors[i][j].cols] != Byte.MAX_VALUE) {

              sectorPts.get(i).get(j).add(portalGraph.addPt(k, sectors[i][j].start.y + sectors[i][j].cols - 1));
              sectorPts.get(i).get(j+1).add(portalGraph.addPt(k, sectors[i][j].start.y + sectors[i][j].cols));

              portalGraph.addEdge(sectorPts.get(i).get(j).get(sectorPts.get(i).get(j).size()-1).name, 
                sectorPts.get(i).get(j+1).get(sectorPts.get(i).get(j+1).size()-1).name);
            }
          }
        }
      }
    }

    // Connect all edges within a sector to each other
    for (int i = 0; i < sectorPts.size(); ++i) {
      for (int j = 0; j < sectorPts.get(i).size(); ++j) {
        ArrayList<Pt> pts = sectorPts.get(i).get(j);
        for (int k = 0; k < pts.size(); ++k) {
          for (int l = 0; l < pts.size(); ++l) {
            if (pts.get(k).name == pts.get(l).name) {
              continue;
            }
            portalGraph.addUndirectedEdge(pts.get(k).name, pts.get(l).name);
          }
        }
      }
    }
  }

  void IncreaseCost(PVector p, byte amount) {
    Point a = CalcFromScreenCoords(p);
    if ((int)this.costField[a.x][a.y] + (int)amount > (int)Byte.MAX_VALUE) {
      this.costField[a.x][a.y] = Byte.MAX_VALUE;
    } else {
      this.costField[a.x][a.y] += amount;
    }
  }

  void DecreaseCost(PVector p, byte amount) {
    Point a = CalcFromScreenCoords(p);
    if (this.costField[a.x][a.y] - amount < 0) {
      this.costField[a.x][a.y] = 0;
    } else {
      this.costField[a.x][a.y] -= amount;
    }
  }

  SectorPath GetSectorPath(Point gridDest) {
    Point sector = GetSector(gridDest);
    Point destInSector = GridToSector(gridDest);
    // Check for an existing cached sector path to the same destination
    if (this.generatedPaths[sector.x][sector.y][destInSector.x][destInSector.y] != null) {
      return this.generatedPaths[sector.x][sector.y][destInSector.x][destInSector.y];
    }

    int[][] integrationField = new int[sectorSize][sectorSize];
    boolean[][] losField = new boolean[sectorSize][sectorSize];
    PVector[][] flowField = new PVector[sectorSize][sectorSize];
    for (int i = 0; i < sectorSize; ++i) {
      for (int j = 0; j < sectorSize; ++j) {
        integrationField[i][j] = 0;
        losField[i][j] = false;
        flowField[i][j] = new PVector(0, 0);
      }
    }

    sectors[sector.x][sector.y].CalcFlowField(destInSector, integrationField, losField, flowField);
    this.generatedPaths[sector.x][sector.y][destInSector.x][destInSector.y] =
      new SectorPath(integrationField, losField, flowField, destInSector);

    return this.generatedPaths[sector.x][sector.y][destInSector.x][destInSector.y];
  }

  GridPath Pathing(Point a, Point b) {
    if (!IsValidSpace(b)) {
      return null;
    }

    ArrayList<Pt> res = portalGraph.aStar(a.x, a.y, b.x, b.y);

    SectorPath[][] sectorPaths = new SectorPath[rows / sectorSize][cols / sectorSize];

    Point destSector = GetSector(b);
    if (res.size() < 2) {
      // We are in the sector already so just generate it
      sectorPaths[destSector.x][destSector.y] = GetSectorPath(b);
    } else {
      Point lastSector = new Point(-1, -1);
      for (int i = 0; i < res.size(); ++i) {
        Point iSector = GetSector(new Point(res.get(i).x, res.get(i).y));
        if (iSector.x == destSector.x && iSector.y == destSector.y) {
          sectorPaths[destSector.x][destSector.y] = GetSectorPath(b);
          lastSector = new Point(destSector.x, destSector.y);
        } else if ((lastSector.x != iSector.x || lastSector.y != iSector.y) && i > 0) {
          sectorPaths[iSector.x][iSector.y] = GetSectorPath(new Point(res.get(i).x, res.get(i).y));
          lastSector = iSector;

          // Point the destination in each sector towards the portal to the next sector
          Point resSector = GetSector(new Point(res.get(i).x, res.get(i).y));
          Point resSectorPoint = GridToSector(new Point(res.get(i).x, res.get(i).y));
          PVector finalMove = new PVector(res.get(i-1).x - res.get(i).x, res.get(i-1).y - res.get(i).y);

          sectorPaths[resSector.x][resSector.y].flowField[resSectorPoint.x][resSectorPoint.y] = finalMove;
        }
      }
    }

    return new GridPath(sectorPaths);
  }

  boolean IsValidSpace(Point p) {
    return p.x >= 0 && p.x < rows && p.y >= 0 && p.y < cols;
  }

  Point CalcFromScreenCoords(PVector point) {
    return new Point((int)(point.x / spaceSizeX), (int)(point.y / spaceSizeY));
  }

  PVector CalcScreenCoordsFromPoint(Point a) {
    return new PVector((float)a.x * spaceSizeX, (float)a.y * spaceSizeY);
  }

  Point SectorToGrid(Point sectorStart, Point p) {
    return new Point(p.x + sectorStart.x, p.y + sectorStart.y);
  }

  Point GridToSector(Point p) {
    Point sector = GetSector(p);
    return new Point(p.x - sectors[sector.x][sector.y].start.x, p.y - sectors[sector.x][sector.y].start.y);
  }

  Point GetSector(Point p) {
    return new Point(p.x / (sectorSize), p.y / (sectorSize));
  }

  void DrawGrid() {
    for (int i = 0; i < rows; ++i) {
      for (int j = 0; j < cols; ++j) {
        if (costField[i][j] < Byte.MAX_VALUE) {
          stroke(costField[i][j] * 2, 255, 0);
          fill(costField[i][j] * 2, 255, 0);
        } else {
          stroke(255, 0, 0);
          fill(255, 0, 0);
        }
        PVector a = CalcScreenCoordsFromPoint(new Point(i, j));
        rect(a.x, a.y, spaceSizeX, spaceSizeY);
      }
    }

    stroke(0);
    for (int i = 1; i <= this.rows; i++) {
      line(i*spaceSizeX, 0, i*spaceSizeX, spaceSizeY * this.cols);
      line(0, i*spaceSizeY, spaceSizeX * this.rows, i*spaceSizeY);
    }

    stroke(0);
    if (DEBUG_ON) {
      portalGraph.draw();
    }
  }
}
