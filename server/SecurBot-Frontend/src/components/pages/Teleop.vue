<template>
  <b-jumbotron id="teleop-layout" class="h-100 " :fluid="true" :container-fluid="true" bg-variant="light">
    <b-row class="h-100">
      <b-col lg="8" md="7" sm="6" class="mh-100">
        <div class="h-100 w-100 m-auto position-relative">
          <video-box VideoId="self-video-stream" :show="showSelf"/>
        </div>
      </b-col>
      <b-col lg="4" md="5" sm="6" class="mh-100">
        <b-row class="h-50 w-100 position-relative m-0">
          <div class="h-100 w-100 m-auto position-relative">
            <video-box VideoId="robot-video-stream" :show="showRobot"/>
          </div>
        </b-row>
        <b-row class="position-relative h-50 m-auto" style="max-width:250px">
          <div class="position-relative m-auto w-100" style="padding-top:100%;height:0;">
            <div class="position-absolute h-100 w-100 border border-secondary rounded-circle shadow-sb" style="top:0;left:0;">
              <joystick :absolute-max-x="1" :absolute-max-y="1" :bus="bus"/>
            </div>
          </div>
        </b-row>
      </b-col>
    </b-row>
  </b-jumbotron>
</template>


<script>
/*
* Author(s):  Edouard Legare <edouard.legare@usherbrooke.ca>
* File :  Layout.vue
* Desc :  Vue SFC used as a page for teleoperation of the robots. This component
*         manages the layout for the teleoperation page. It uses 2 VideoBox components
*         and 1 joystick component. The bigger VideoBox is used to show the map and 
*         the smaller the video feed from the camera on the robot. The joystick is used
*         to send command to the robot for manual control.  
*         It communicates with parent component through the bus in props.
*
* Dependencies : 
*       -VideoBox.vue
*       -Joystick.vue
*       -Bootstrap-Vue
*
*/
import VideoBox from "../widget/VideoBox.vue";
import Joystick from "../widget/Joystick.vue";

export default {
  name:'teleop-page',
  props:['bus','router'],
  components:{
    VideoBox,
    Joystick
  },
  mounted(){
    console.log("Teleop have been mounted");
    this.router.$emit('mounted');
  },
  destroyed(){
    console.log("Teleop have been destroyed");
    this.router.$emit('destroyed');
  },
  data(){
    return{
      showRobot:true,
      showSelf:true,
    }
  }
  
}
</script>

<style>
</style>

