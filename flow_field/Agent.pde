// General constants
float AGENT_RADIUS = 2;
float SLOW_RADIUS = 6;
float TARGET_RADIUS = 2;
float MAX_SPEED = 12;
float MAX_ACCELERATION = 1;

// Rotation
float SLOW_ROTATION_RADIUS = PI/10;
float TARGET_ROTATION_RADIUS = PI/20;
float MAX_ANGULAR_SPEED = PI/90;

// Repel from leader
float THRESH = 6;
float CONSTANT = 5;

float FOLLOWER_PATHING_DISTANCE = 15;

class Agent {
  boolean selected;
  boolean reachedDest;
  PVector destination;
  private GridPath path;

  private Body body;
  private Fixture fixture;
  float orientation;

  boolean isAvoiding = false;
  boolean avoidanceDir = false;

  Agent(PVector startPos) {
    // Define a polygon
    PolygonShape sd = new PolygonShape();
    // Circles in box2d processing gave assertion errors, so this is close enough
    sd.setAsBox(box2d.scalarPixelsToWorld(AGENT_RADIUS*2), box2d.scalarPixelsToWorld(AGENT_RADIUS*2));

    // Define a fixture
    FixtureDef fd = new FixtureDef();
    fd.shape = sd;
    // Parameters that affect physics
    fd.density = 20.0;
    fd.friction = 0.0;
    fd.restitution = 0.0;

    // Define the body and make it from the shape
    BodyDef bd = new BodyDef();
    bd.type = BodyType.DYNAMIC;
    bd.position.set(box2d.coordPixelsToWorld(startPos));

    body = box2d.createBody(bd);
    this.body.setUserData(this);
    this.fixture = this.body.createFixture(fd);

    this.orientation = 0;

    this.selected = false;
    this.reachedDest = true;
    this.destination = startPos;
  }

  void SetDest(PVector dest, GridPath path) {
    this.reachedDest = false;
    this.destination = dest;
    this.path = path;
  }

  PVector GetPos() {
    return box2d.getBodyPixelCoordPVector(this.body);
  }

  boolean IsIn(PVector a, PVector b) {
    PVector pos = GetPos();
    if (a.x < b.x && a.y < b.y) {
      return pos.x >= a.x && pos.x <= b.x && pos.y >= a.y && pos.y <= b.y;
    } else if (a.x > b.x && a.y < b.y) {
      return pos.x <= a.x && pos.x >= b.x && pos.y >= a.y && pos.y <= b.y;
    } else if (a.x < b.x && a.y > b.y) {
      return pos.x >= a.x && pos.x <= b.x && pos.y <= a.y && pos.y >= b.y;
    } else if (a.x > b.x && a.y > b.y) {
      return pos.x <= a.x && pos.x >= b.x && pos.y <= a.y && pos.y >= b.y;
    }
    return false;
  }

  void GridPathing() {
    PVector pos = GetPos();
    Point floor = grid.CalcFromScreenCoords(pos);

    if (grid.IsValidSpace(floor)) {
      if (floor.equals(grid.CalcFromScreenCoords(this.destination))) {
        // We are at same grid, so steer directly towards destination
        Steering(new PVector(this.destination.x - pos.x, this.destination.y - pos.y), pos);
        return;
      }

      Point sector = grid.GetSector(floor);
      Point sectorFloor = grid.GridToSector(floor);
      if (path == null || path.sectorPaths[sector.x][sector.y] == null) {
        // Re-calculate path if we go off track
        this.SetDest(this.destination, 
          grid.Pathing(floor, grid.CalcFromScreenCoords(this.destination)));
        // Still no grid path, so blind steer
        if (path == null || path.sectorPaths[sector.x][sector.y] == null) {
          Steering(new PVector(this.destination.x - pos.x, this.destination.y - pos.y), pos);
          return;
        }
      }

      if (!path.sectorPaths[sector.x][sector.y].sectorDest.equals(sectorFloor) &&
        path.sectorPaths[sector.x][sector.y].losField[sectorFloor.x][sectorFloor.y]) {
        // Navigate straight towards destination in sector if we have LOS
        PVector sDest = grid.CalcScreenCoordsFromPoint(grid.SectorToGrid(grid.sectors[sector.x][sector.y].start, 
          path.sectorPaths[sector.x][sector.y].sectorDest));
        sDest.add(grid.spaceSizeX / 2, grid.spaceSizeY / 2);
        Steering(new PVector(sDest.x - pos.x, sDest.y - pos.y), pos);
        return;
      }

      // Get direction of movement from flow field
      PVector dir = new PVector(path.sectorPaths[sector.x][sector.y].flowField[sectorFloor.x][sectorFloor.y].x, 
        path.sectorPaths[sector.x][sector.y].flowField[sectorFloor.x][sectorFloor.y].y);
      Steering(dir, pos);
      return;
    }

    // Blind steer towards grid if we go off screen
    Steering(new PVector(this.destination.x - pos.x, this.destination.y - pos.y), pos);
  }

  void FollowerPathing(PVector leaderPos) {
    PVector pos = GetPos();
    if (dist(this.destination.x, this.destination.y, pos.x, pos.y) > FOLLOWER_PATHING_DISTANCE) {
      // Too far away, so we should use real pathfinding to get back to formation
      GridPathing();
      return;
    }
    PVector dir = new PVector(this.destination.x - pos.x, this.destination.y - pos.y);

    // Repel from leader so as not to run them over
    PVector leaderDir = new PVector(pos.x - leaderPos.x, pos.y - leaderPos.y);
    float distanceFromRepel = leaderDir.mag();
    if (distanceFromRepel < THRESH) {
      float strength = CONSTANT / distanceFromRepel * distanceFromRepel;
      leaderDir.normalize();
      leaderDir.mult(strength);
      dir.add(leaderDir);
    }

    Steering(dir, pos);
  }

  private void Steering(PVector dir, PVector pos) {
    if (dir.x == 0 && dir.y == 0) {
      return;
    }
    dir.normalize();

    // Rough orientation setting is just for formation setting purposes
    float targetOrientation = atan2(dir.x, dir.y);
    float rotation = targetOrientation - orientation;
    float rotationSize = abs(rotation);
    // Calculate target rotation
    float targetRotation = 0;
    if (rotationSize > SLOW_ROTATION_RADIUS) {
      targetRotation = MAX_ANGULAR_SPEED;
    } else if (rotationSize > TARGET_ROTATION_RADIUS) {
      targetRotation = MAX_ANGULAR_SPEED * rotationSize / SLOW_ROTATION_RADIUS;
    }
    // Set direction
    targetRotation *= rotation / rotationSize;
    // Update orientation
    orientation += targetRotation;

    // Determine target speed
    float targetSpeed = 0;
    if (!reachedDest) {
      float distance = dist(this.destination.x, this.destination.y, pos.x, pos.y);
      if (distance > SLOW_RADIUS) {
        targetSpeed = MAX_SPEED;
      } else if (distance > TARGET_RADIUS) {
        targetSpeed = MAX_SPEED * distance / (float) SLOW_RADIUS;
      } else {
        this.reachedDest = true;
      }
    }

    // Slow down the agent based on grid cost
    Point floor = grid.CalcFromScreenCoords(pos);
    if (grid.IsValidSpace(floor)) {
      targetSpeed -= ((float)grid.costField[floor.x][floor.y] / (float)Byte.MAX_VALUE) * targetSpeed;
    }

    // Dir is in screen coords, we need box2d coords
    dir.y *= -1;
    // Determine target velocity
    dir.mult(targetSpeed);

    // Determine acceleration
    PVector agentAcceleration = new PVector(dir.x, dir.y);
    agentAcceleration.sub(this.body.getLinearVelocity().x, this.body.getLinearVelocity().y);
    Vec2 avoidanceForce = CollisionAvoidance();
    if (avoidanceForce != null) {
      agentAcceleration.add(avoidanceForce.x, avoidanceForce.y);
    }

    // Clip acceleration
    if (agentAcceleration.mag() > MAX_ACCELERATION) {
      agentAcceleration.normalize();
      agentAcceleration.mult(MAX_ACCELERATION);
    }

    // Update velocity
    this.body.setLinearVelocity(new Vec2(this.body.getLinearVelocity().x + agentAcceleration.x, 
      this.body.getLinearVelocity().y + agentAcceleration.y));
  }

  // Processing does not support lambdas, so this workaround is required
  class AgentRayCastCallback implements RayCastCallback {
    Agent agent;
    float minFraction;
    Fixture closestFixture;

    AgentRayCastCallback(Agent agent, float minFraction) {
      this.agent = agent;
      this.minFraction = minFraction;
      this.closestFixture = null;
    }

    public float reportFixture(Fixture fixture, Vec2 point, Vec2 normal, float fraction) {
      //Ignore ourself
      if (fixture == agent.fixture) {
        return fraction;
      }
      if (fraction < minFraction &&
        (fixture.getBody().getType() == BodyType.DYNAMIC || fixture.getBody().getType() == BodyType.STATIC)) {
        minFraction = fraction;
        closestFixture = fixture;
      }
      return 0;
    }
  }
  // Collision avoidance function inspired by howtorts.github.io/2014/01/14/avoidance-behaviours.html
  private Vec2 CollisionAvoidance() {
    if (this.body.getLinearVelocity().lengthSquared() <= AGENT_RADIUS) {
      this.isAvoiding = false;
      return null;
    }

    AgentRayCastCallback callback = new AgentRayCastCallback(this, 2);
    box2d.world.raycast(callback, this.body.getPosition(), this.body.getPosition().add(this.body.getLinearVelocity()));
    if (callback.closestFixture == null) {
      this.isAvoiding = false;
      return null;
    }

    if (callback.closestFixture.getBody().getType() == BodyType.STATIC) {
      this.isAvoiding = true;
      return this.body.getPosition().sub(callback.closestFixture.getBody().getPosition()).mulLocal(1/callback.minFraction);
    }

    Vec2 resultVector = null;
    Body collisionBody = callback.closestFixture.getBody();
    float ourVelocityLengthSquared = this.body.getLinearVelocity().lengthSquared();
    Vec2 combinedVelocity = this.body.getLinearVelocity().add(collisionBody.getLinearVelocity());
    float combinedVelocityLengthSquared = combinedVelocity.lengthSquared();

    // We are going in the same direction and they aren't avoiding
    if (combinedVelocityLengthSquared > ourVelocityLengthSquared &&
      ((Agent)callback.closestFixture.getBody().getUserData()).isAvoiding == false) {
      this.isAvoiding = false;
      return null;
    }

    Vec2 vectorInOtherDirection = callback.closestFixture.getBody().getPosition().sub(this.body.getPosition());

    boolean isLeft;
    if (((Agent)callback.closestFixture.getBody().getUserData()).isAvoiding == true) {
      isLeft = ((Agent)callback.closestFixture.getBody().getUserData()).avoidanceDir;
    } else {
      float dot = this.body.getLinearVelocity().x * -vectorInOtherDirection.y + this.body.getLinearVelocity().y * vectorInOtherDirection.x;
      isLeft = dot > 0;
    }
    this.isAvoiding = true;
    this.avoidanceDir = isLeft;

    resultVector = isLeft ?
      new Vec2(-vectorInOtherDirection.y, vectorInOtherDirection.x) :
      new Vec2(vectorInOtherDirection.y, -vectorInOtherDirection.x);
    resultVector.normalize();

    resultVector.mulLocal(AGENT_RADIUS * 2).mulLocal(1/callback.minFraction);
    return resultVector;
  }

  // For drawing
  private void SetTransform(PVector translate, float scale, float rot) {
    pushMatrix();
    translate(translate.x, translate.y);
    rotate(radians(rot));
    scale(scale);
  }
  void DrawAgent() {
    if (DEBUG_ON && selected) {
      if (path != null) {
        for (int row = 0; row < grid.rows; ++row) {
          for (int col= 0; col < grid.cols; ++col) {
            stroke(0);
            fill(0);
            Point sector = grid.GetSector(new Point(row, col));
            Point sectorPoint = grid.GridToSector(new Point(row, col));
            if (path.sectorPaths[sector.x][sector.y] != null) {
              if (path.sectorPaths[sector.x][sector.y].losField[sectorPoint.x][sectorPoint.y]) {
                stroke(0, 0, 255);
                fill(128);
                PVector a = grid.CalcScreenCoordsFromPoint(grid.SectorToGrid(grid.sectors[sector.x][sector.y].start, sectorPoint));
                rect(a.x, a.y, grid.spaceSizeX, grid.spaceSizeY);
              }
              stroke(0);
              fill(0);
              text(Float.toString(path.sectorPaths[sector.x][sector.y].integrationField[sectorPoint.x][sectorPoint.y]), 
                row*grid.spaceSizeX + grid.spaceSizeX/2, col*grid.spaceSizeY + grid.spaceSizeY/2);
              line(row*grid.spaceSizeX + grid.spaceSizeX/2, col*grid.spaceSizeY + grid.spaceSizeY/2, 
                row*grid.spaceSizeX + grid.spaceSizeX/2 + (path.sectorPaths[sector.x][sector.y].flowField[sectorPoint.x][sectorPoint.y].x * 25), 
                col*grid.spaceSizeY + grid.spaceSizeY/2 + (path.sectorPaths[sector.x][sector.y].flowField[sectorPoint.x][sectorPoint.y].y * 25));
            }
          }
        }
      }
      stroke(255, 255, 255);
      fill(255, 255, 255);
      ellipse(destination.x, destination.y, 2, 2);
    }

    this.SetTransform(box2d.getBodyPixelCoordPVector(this.body), AGENT_RADIUS * 2, 0);
    if (selected) {
      stroke(0, 0, 255);
    } else {
      stroke(255, 0, 0);
    }
    ellipse(0, 0, 1, 1);
    popMatrix();
  }
}

// Function to check contacts and tell agents they have reached
// their destination if they hit another agent with the same
// destination as them that has already reached the destination
void CheckContacts() {
  for (Contact cp = box2d.world.getContactList(); cp != null; cp = cp.getNext()) {
    // Get both shapes
    Fixture f1 = cp.getFixtureA();
    Fixture f2 = cp.getFixtureB();
    // Get both bodies
    Body b1 = f1.getBody();
    Body b2 = f2.getBody();

    // Get our objects that reference these bodies
    Object o1 = b1.getUserData();
    Object o2 = b2.getUserData();
    if (o1 == null || o2 == null) {
      return;
    }

    if (o1.getClass() == Agent.class && o2.getClass() == Agent.class) {
      Agent p1 = (Agent) o1;
      Agent p2 = (Agent) o2;

      if (p1.destination.equals(p2.destination) && (p1.reachedDest || p2.reachedDest)) {
        p1.reachedDest = true;
        p2.reachedDest = true;
      }
    }
  }
}
