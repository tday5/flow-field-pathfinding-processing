# Flow Field Pathfinding in Processing

## Background:
This is a project I wrote for a class in Game AI. The idea was to focus on implementing a version of a flow field
pathfinding algorithm as described in this [paper](http://www.gameaipro.com/GameAIPro/GameAIPro_Chapter23_Crowd_Pathfinding_and_Steering_Using_Flow_Field_Tiles.pdf). Java with Processing was the common language used in the class, as it offers a quick and easy library for visual output, so that is what I used here. I believe the flow field implementation here is pretty successful, as well as some of the "extra features" such as formations, but individual unit steering does leave something to be desired. This, however, was not the focus of the project and is really here to aid in providing a visual demonstration of the algorithm.

### Some Quick Features:
- Flow field pathfinding which subdivides the map into "sectors" of a configurable size
  - A per sector flow field cache that is shared among all units, which means units navigating in a similar manner through
    a given sector can use the same calculated flow field
  - A* search to chain sectors for unit navigation
- Navigation optimizes for differences in terrain "height", or difficulty of moving through that terrain
  - Also supports navigating around impassable obstacles
- A formation system that allows grouping arbitrary numbers of units together with a specified separation pattern
  - The formation system is very modular, and adding new formations requires a single new function
  - Formations are dynamic and will reform correctly if units are added or removed
- Units have a very simple collision avoidance behavior that sends out a single raycast and applies an avoidance force if
  the unit will collide with something

## Instructions:
The whole program is written in Processing, so that should make it pretty easy to run on any platform. Simply download
Processing at https://processing.org/download/. The only additional library I used was Box2D for Processing, which can be
downloaded by going to the “Sketch” tab in the Processing IDE, then to “Import Library”, and lastly “Add Library...”. This
should open a new window to help import external Processing libraries. Box2D can be found by typing “Box2D” into the filter,
it is the library titled [“Box2D for Processing” by Daniel Shiffman](https://github.com/shiffman/Box2D-for-Processing).

## Controls:
- ’t’ to toggle terrain edit mode. This mode will allow editing of terrain by clicking and dragging the
mouse. Yellow terrain signifies higher cost, and red signifies impassable. A right click allows
decreasing the cost of terrain. *NOTE*: toggling terrain mode will clear all active agents.
- Hold ‘a’ and left click to add units when not in terrain edit mode, or alternately click and hold and
press ‘a’. Processing’s event detection ordering can be finicky.
- Left click and drag to create a selection box around units.
- Right click on a location to have units move there.
- With units selected, press ‘1’ to have the units form a ‘Box’ formation.
- With units selected, press ‘2’ to have the units form a ‘Line’ formation.
- With units selected, press ‘3’ to have the units form a ‘V’ formation.
- ‘d’ to toggle debug mode. *NOTE*: this can cause severe slowdown due to rendering
overhead when there are lots of grid tiles or units.
- ‘q’ to toggle the rough A* equivalent mode. (Sets the sector size to 1, essentially telling the
flow field system just to run A* on the portal graph without creating any actual flow fields.) *NOTE*: toggling A* mode will clear all active agents. *NOTE 2*: Box2D doesn’t seem to like the amount of computation going on when running A* mode with lots of units, so may have a higher rate of slowdown/errors.