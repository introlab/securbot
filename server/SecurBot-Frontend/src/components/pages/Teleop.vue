<template>
  <b-jumbotron id="teleop-layout" class="h-100 " :fluid="true" :container-fluid="true" bg-variant="light">
    <b-row class="teleop-row">
      <b-col lg="8" md="7" sm="6" class="mh-100">
        <div class="video-container">
          <video-box VideoId="self-video-stream" :show="showSelf"/>
        </div>
      </b-col>
      <b-col lg="4" md="5" sm="6" class="mh-100">
        <b-row class="robot-video-row">
          <div class="video-container">
            <video-box VideoId="robot-video-stream" :show="showRobot"/>
          </div>
        </b-row>
        <b-row class="joystick-row">
          <div class="outer-joystick-container">
            <div class="joystick-container">
              <joystick :absolute-max-x="1" :absolute-max-y="1" :bus="bus"/>
            </div>
          </div>
        </b-row>
      </b-col>
    </b-row>
  </b-jumbotron>
</template>


<script>
import VideoBox from "../operator/VideoBox.vue";
import Joystick from "../widget/Joystick.vue";

export default {
  name:'teleop-page',
  props:['bus'],
  components:{
    VideoBox,
    Joystick
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
.video-container{
  width: 100%;
  height: 100%;
  margin: auto;
  position: relative;
}
.teleop-row{
  height: 100%;
  max-height: calc(100vh - 64px - 128px);
}
.robot-video-row{
  height: 50%;
  width: 100%;
  position: relative;
  margin: 0px;
  margin-bottom: 20px;
}
.joystick-row{
  position: relative;
  margin: auto;
  height: calc(50% - 20px);
  max-width: 250px;
}
.outer-joystick-container{
  position: relative;
  padding-top: 100%;
  margin: auto;
  width: 100%;
  height: 0;
}
.joystick-container{
  position: absolute;
  bottom: 0;
  left: 0;
  width: 100%;
  height : 100%;
  border: 2px solid dimgray;
  border-radius: 50%;
  box-shadow: 0 4px 8px 0 rgba(0, 0, 0, 0.5), 0 6px 20px 0 rgba(0, 54, 5, 0.19);
}
</style>

