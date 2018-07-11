// Formations
final int NONE = 0;
final int BOX = 1;
final int LINE = 2;
final int VEE = 3;

// Spacing
final int DIRECT_SPACING = 12;
final int DIAGONAL_SPACING = 8;

class Group {
  int curFormation;
  ArrayList<Agent> agents;

  Group(int formation, ArrayList<Agent> agents) {
    this.agents = agents;
    SetFormation(formation);
  }

  void RmAgents(ArrayList<Agent> rmAgents) {
    for (int i = 0; i < rmAgents.size(); ++i) {
      if (agents.contains(rmAgents.get(i))) {
        agents.remove(rmAgents.get(i));
      }
    }
  }

  void AddAgent(Agent a) {
    this.agents.add(a);
  }

  void SetDest(PVector dest) {
    if (this.curFormation == NONE) {
      for (int i = 0; i < agents.size(); ++i) {
        agents.get(i).SetDest(dest, 
          grid.Pathing(grid.CalcFromScreenCoords(agents.get(i).GetPos()), grid.CalcFromScreenCoords(dest)));
      }
      return;
    }

    // Pathfind the leader only
    agents.get(0).SetDest(dest, 
      grid.Pathing(grid.CalcFromScreenCoords(agents.get(0).GetPos()), grid.CalcFromScreenCoords(dest)));
    SetFollowDestinations();
  }

  void Move() {
    if (this.curFormation == NONE) {
      for (int i = 0; i < agents.size(); ++i) {
        agents.get(i).GridPathing();
      }
      return;
    }
    agents.get(0).GridPathing();
    
    SetFollowDestinations();
    for (int i = 1; i < agents.size(); ++i) {
      agents.get(i).FollowerPathing(agents.get(0).GetPos());
    }
  }
  private void SetFollowDestinations() {
    if (this.curFormation == NONE) {
      return;
    }
    for (int i = 1; i < agents.size(); ++i) {
      PVector nextPos = NextPosition(this.curFormation, i);
      float s = sin(agents.get(0).orientation);
      float c = cos(agents.get(0).orientation);
      nextPos = new PVector((nextPos.x * c) - (nextPos.y * s), (nextPos.x * s) + (nextPos.y * c));

      nextPos.add(agents.get(0).GetPos());
      agents.get(i).SetDest(nextPos, null);
    }
  }

  void SwapAgents(int a, int b) {
    Agent aa = agents.get(a);
    agents.set(a, agents.get(b));
    agents.set(b, aa);
  }

  void SetFormation(int f) {
    this.curFormation = f;
    if (f == NONE) {
      return;
    }

    // Get leader and move him to index 0
    float totalX = 0;
    float totalY = 0;
    for (int i = 0; i < agents.size(); ++i) {
      totalX += agents.get(i).GetPos().x;
      totalY += agents.get(i).GetPos().y;
    }
    PVector centerPos = new PVector(totalX / agents.size(), totalY / agents.size());
    int leader = GetUnitClosestTo(centerPos, 0);
    SwapAgents(leader, 0);
    agents.get(0).SetDest(centerPos, null);

    for (int i = 1; i < agents.size(); ++i) {
      PVector nextPos = NextPosition(f, i);
      float s = sin(agents.get(0).orientation);
      float c = cos(agents.get(0).orientation);
      nextPos = new PVector(nextPos.x * c - nextPos.y * s, nextPos.x * s + nextPos.y * c);
      nextPos.add(agents.get(0).GetPos());

      int nextUnit = GetUnitClosestTo(nextPos, i);
      SwapAgents(nextUnit, i);
      agents.get(i).SetDest(nextPos, null);
    }
  }

  private int GetUnitClosestTo(PVector pos, int from) {
    float closestDist = Float.MAX_VALUE;
    int closest = from;
    for (int i = from; i < agents.size(); ++i) {
      PVector aPos = agents.get(i).GetPos();
      float dist = dist(aPos.x, aPos.y, pos.x, pos.y);
      if (dist < closestDist) {
        closestDist = dist;
        closest = i;
      }
    }
    return closest;
  }

  private PVector NextPosition(int f, int unit) {
    switch(f) {
    case BOX:
      return Box(unit);
    case LINE:
      return Line(unit);
    case VEE:
      return Vee(unit);
    default:
      return new PVector(0, 0);
    }
  }

  // All formation functions return an offset from the leader
  private PVector Box(int unit) {
    int multiple = ((unit-1) / 8)+1;
    int counter = (unit-1) % 8;
    switch(counter) {
    case 0:
      return new PVector(multiple * DIRECT_SPACING, 0);
    case 1:
      return new PVector(multiple * DIRECT_SPACING, multiple * DIRECT_SPACING);
    case 2:
      return new PVector(0, multiple * DIRECT_SPACING);
    case 3:
      return new PVector(multiple * -DIRECT_SPACING, multiple * DIRECT_SPACING);
    case 4:
      return new PVector(multiple * -DIRECT_SPACING, 0);
    case 5:
      return new PVector(multiple * -DIRECT_SPACING, multiple * -DIRECT_SPACING);
    case 6:
      return new PVector(0, multiple * -DIRECT_SPACING);
    case 7:
      return new PVector(multiple * DIRECT_SPACING, multiple * -DIRECT_SPACING);
    default:
      return new PVector(0, 0);
    }
  }

  private PVector Line(int unit) {
    if (unit % 2 == 0) {
      return new PVector((unit-(unit/2)) * DIRECT_SPACING, 0);
    }
    return new PVector((unit-(unit/2)) * -DIRECT_SPACING, 0);
  }

  private PVector Vee(int unit) {
    if (unit % 2 == 0) {
      return new PVector((unit-(unit/2)) * DIRECT_SPACING, (unit/2) * -DIAGONAL_SPACING);
    }
    return new PVector((unit-(unit/2)) * -DIRECT_SPACING, ((unit/2)+1) * -DIAGONAL_SPACING);
  }
}
