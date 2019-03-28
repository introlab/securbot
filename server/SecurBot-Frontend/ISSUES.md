# Issues encountered and other problems

## Routing :
  * When switching pages, the video feeds are not reset to the videobox, leaving a black box.
  * Waypoint sometimes doesn't appear where clicked. Seems to be a problem with the canvas resizing.
  * The waypoint table changes its height if there is too much waypoint, restraining the height doesn't seem to fix the issue. **Fixed** This bug seem to have fix itself when switching from the javascript generated table to the vue one.
  * When resizing:
     * The video boxes do not respect sizing, either height or width take priority and make the video bigger than the element that contains it. [**Fixed**] Establishing a priority (either height or width) and setting a max for the other seems to fix the issue.
     * Since its position and max are in absolute, resizing the page makes the joystick overflow. [**Workaround**] Making the absolute small enough and setting the margin to auto ''fix'' the issue. It's not actually fixed since the margin doesn't stop the overflow, but it work for now, will need to be looked into.
     * Because of set heights, resizing horizontaly doesn't look good and the video boxes are juste weird.
     * The UI doesn't work on mobile
     * There seems to be an inconsistent bug with the videos where it is not centered in the black box. It was thought to be a firefox issue, but it also happen on a chromium browser. Strange bug...