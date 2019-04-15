<template>
  <b-jumbotron
    id="teleop-layout"
    :fluid="true"
    :container-fluid="true"
    class="h-100 "
    bg-variant="light">
    <b-row class="h-100">
      <b-col
        lg="8"
        md="7"
        sm="6"
        class="mh-100">
        <div class="h-100 w-100 m-auto position-relative">
          <video-box
            :show="showCamera"
            video-id="camera-stream" />
        </div>
      </b-col>
      <b-col
        lg="4"
        md="5"
        sm="6"
        class="mh-100">
        <b-row class="h-50 w-100 position-relative m-0">
          <div class="h-100 w-100 m-auto position-relative">
            <video-box
              :show="showMap"
              video-id="map-stream" />
          </div>
        </b-row>
        <b-row
          class="position-relative h-50 m-auto"
          style="max-width:calc(100vh*0.2)">
          <div
            class="position-relative m-auto w-100"
            style="padding-top:100%;height:0;">
            <div
              class="position-absolute h-100 w-100 border border-secondary rounded-circle shadow-sb"
              style="top:0;left:0;">
              <joystick
                :enable="enableJoystick"
                :absolute-max-x="1"
                :absolute-max-y="1"
                :bus="bus" />
            </div>
          </div>
        </b-row>
      </b-col>
    </b-row>
  </b-jumbotron>
</template>


<script>
import Vue from 'vue';

import VideoBox from '../widget/VideoBox';
import Joystick from '../widget/Joystick';

/**
 * @vuese
 * @group Pages
 * @author Edouard Legare <edouard.legare@usherbrooke.ca>
 * @version 1.0.0
 *
 * Description : Component used as a page for teleoperation of the robot. It manages the layout of
 * its components and communicate with its parent component through a bus given in props. The
 * components used in this page are 2 VideoBox and 1 Joystick.
 *
 * This component have the following dependency : VideoBox.vue Component, Joystick.vue Component
 * and Bootstrap-Vue for styling.
 */
export default {
  name: 'teleop-page',
  components: {
    VideoBox,
    Joystick,
  },
  props: {
    /**
     * Vue bus use to communicate events to the other components
     */
    bus: {
      type: Vue,
      required: true,
    },
    /**
     * Vue bus use to emit events to the parent component for routing purposes
     */
    router: {
      type: Vue,
      required: true,
    },
  },
  data() {
    return {
      showCamera: true,
      showMap: true,
      enableJoystick: false,
    };
  },
  mounted() {
    console.log('Teleop have been mounted');
    this.init();
  },
  destroyed() {
    console.log('Teleop have been destroyed');
    /**
     * Destroyed event
     * @arg Does not take any parameter
     */
    this.router.$emit('destroyed');
  },
  methods: {
    /**
     * @vuese
     * Init function for the teleop page
     * @arg Does not take any parameter
    */
    init() {
      // Triggered when button is clicked
      // @arg This event doesn't emit any argument
      this.bus.$emit('mounted');
      this.bus.$on('on-joystick-state-changed', this.changeJoystickState);
    },
    /**
     * @vuese
     * Joystick state event callback, used to change the joystick state
     *
     * @arg The argument is a boolean representing the state
    */
    changeJoystickState(state) {
      if (state === 'enable') {
        this.enableJoystick = true;
      } else {
        this.enableJoystick = false;
      }
    },
  },

};
</script>

<style>
</style>
