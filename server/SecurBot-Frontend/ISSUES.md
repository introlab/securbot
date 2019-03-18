# Issues encountered and other problems

## Routing :
  * When switching pages, the video feeds are not reset to the videobox, leaving a black box.
  * Waypoint sometimes doesn't appear where clicked. Seems to be a problem with the canvas resizing.
  * The waypoint table changes its height if there is too much waypoint, restraining the height doesn't seem to fix the issue. **Fixed** This bug seem to have fix itself when switching from the javascript generated table to the vue one.