//Controls:
//- ’t’ to toggle terrain mode. This mode will allow editing of terrain by clicking
//  and dragging the mouse. Yellow terrain signifies higher cost, and red signifies
//  impassable. A right click allows decreasing the cost of terrain. NOTE: toggling
//  terrain mode will clear all active agents.
//- Hold ‘a’ and left click to add units when not in terrain mode, or alternately
//  click and hold and press ‘a’. Processing’s event detection ordering can be finicky.
//- Left click and drag to create a selection box around units.
//- Right click on a location to have units more there.
//- With units selected, press ‘1’ to have the units form a ‘Box’ formation.
//- With units selected, press ‘2’ to have the units form a ‘Line’ formation.
//- With units selected, press ‘3’ to have the units form a ‘V’ formation.
//- ‘d’ to toggle debug mode. NOTE: this can cause severe slowdown due to rendering
//  overhead when there are lots of grid tiles or units.
//- ‘q’ to toggle the rough A* equivalent mode. (Sets the sector size to 1, essentially
//  telling the flow field system just to run A* on the portal graph without creating
//  any actual flow fields.) NOTE: toggling A* mode will clear all active agents.
//  NOTE 2: Box2D doesn’t seem to like the amount of computation going on when running
//  A* mode with lots of units, so may have a higher rate of slowdown/errors.

// Using Box2D for Processing library found at github.com/shiffman/Box2D-for-Processing
import shiffman.box2d.*;
import org.jbox2d.common.*;
import org.jbox2d.dynamics.joints.*;
import org.jbox2d.collision.shapes.*;
import org.jbox2d.collision.shapes.Shape;
import org.jbox2d.common.*;
import org.jbox2d.dynamics.*;
import org.jbox2d.dynamics.contacts.*;
import org.jbox2d.callbacks.*;

import java.util.*;

int RES_X = 800;
int RES_Y = 600;

Box2DProcessing box2d;

Grid grid;
ArrayList<Agent> agents;
ArrayList<Group> groups;
Group selectedGroup;

boolean selecting = false;
PVector selectingStart;

boolean DEBUG_ON = false;
boolean TERRAIN_ON = false;
boolean A_STAR_EQUIV_ON = false;
int NORMAL_SECTOR_SIZE = 10;

void settings() {
  size(RES_X, RES_Y);
}

void setup() {
  // Box2D setup
  box2d = new Box2DProcessing(this);
  box2d.createWorld(new Vec2(0, 0));

  grid = new Grid(50, 50, NORMAL_SECTOR_SIZE);
  agents = new ArrayList<Agent>();
  groups = new ArrayList<Group>();
  selectedGroup = null;
}

void mouseClicked() {
  if (TERRAIN_ON) {
    return;
  }
  if (keyPressed && key == 'a') {
    agents.add(new Agent(new PVector(mouseX, mouseY)));
  } else if (mouseButton != LEFT) {
    if (selectedGroup == null) {
      setNewGroup(0);
    }
    // Still might be null if no agents selected
    if (selectedGroup != null) {
      selectedGroup.SetDest(new PVector(mouseX, mouseY));
    }
  }
}

void mousePressed() {
  if (!TERRAIN_ON && mouseButton == LEFT) {
    selecting = true;
    selectingStart = new PVector(mouseX, mouseY);
  }
}

void mouseReleased() {
  if (selecting) {
    selecting = false;
    PVector selectingEnd = new PVector(mouseX, mouseY);
    for (int i = 0; i < agents.size(); ++i) {
      if (agents.get(i).IsIn(selectingStart, selectingEnd)) {
        agents.get(i).selected = true;
      } else {
        agents.get(i).selected = false;
      }
    }
    selectedGroup = null;
  }
}

void clearAgents() {
  for (int i = 0; i < agents.size(); ++i) {
    box2d.world.destroyBody(agents.get(i).body);
  }
  agents.clear();
  groups.clear();
  selectedGroup = null;
}

void setNewGroup(int formation) {
  ArrayList<Agent> selected = new ArrayList<Agent>();
  for (int i = 0; i < agents.size(); ++i) {
    if (agents.get(i).selected) {
      selected.add(agents.get(i));
    }
  }
  if (selected.isEmpty()) {
    return;
  }
  // Remove all agents we want to re-group from any other groups
  // they might be in
  for (int i = groups.size()-1; i >= 0; --i) {
    groups.get(i).RmAgents(selected);
    if (groups.get(i).agents.isEmpty()) {
      groups.remove(i);
    }
  }
  selectedGroup = new Group(formation, selected);
  groups.add(selectedGroup);
}
void keyPressed() {
  if (key == 'd') {
    DEBUG_ON = !DEBUG_ON;
  }
  if (key == 't') {
    if (TERRAIN_ON) {
      grid.CalcPortalGraph();
    } else {
      clearAgents();
    }
    TERRAIN_ON = !TERRAIN_ON;
  }
  if (key == 'q') {
    clearAgents();
    if (A_STAR_EQUIV_ON) {
      grid.PrepSectors(NORMAL_SECTOR_SIZE);
    } else {
      grid.PrepSectors(1);
    }
    A_STAR_EQUIV_ON = !A_STAR_EQUIV_ON;
  }
  if (key == '1') {
    setNewGroup(1);
  } else if (key == '2') {
    setNewGroup(2);
  } else if (key == '3') {
    setNewGroup(3);
  }
}

void draw() {
  if (!TERRAIN_ON) {
    box2d.step();
    CheckContacts();
    for (int i = 0; i < groups.size(); ++i) {
      groups.get(i).Move();
    }
  } else if (mousePressed) {
    byte amount = 10;
    if (mouseButton == LEFT) {
      grid.IncreaseCost(new PVector(mouseX, mouseY), amount);
    } else if (mouseButton == RIGHT) {
      grid.DecreaseCost(new PVector(mouseX, mouseY), amount);
    }
  }

  background(128);
  grid.DrawGrid();
  for (int i = 0; i < agents.size(); ++i) {
    agents.get(i).DrawAgent();
  }
  if (selecting) {
    noFill();
    stroke(0, 0, 255);
    rect(selectingStart.x, selectingStart.y, mouseX - selectingStart.x, mouseY - selectingStart.y);
  }

  if (TERRAIN_ON) {
    stroke(0);
    fill(0);
    text("Terrain edit mode on", 30, 20);
  }
}
