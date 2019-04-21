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
/**
 * Component used as a page for teleoperation of the robot. It manages the layout of
 * its components and communicate with its parent component through a bus given in props. The
 * components used in this page are 2 VideoBox and 1 Joystick. This component have the following
 * dependency : VideoBox.vue Component, Joystick.vue Component and Bootstrap-Vue for styling.
 *
 *
 * @module Teleop
 * @vue-prop {Vue} bus - Vue bus use to emit event to other components.
 * @vue-prop {Vue} Router - Vue bus use to routing emit event to parent.
 * @vue-event {} destroyed - Event indicating the component has been destroyed.
 * @vue-event {} mounted - Event indicating the component has been mounted.
 * @vue-data {boolean} showCamera - Enable or disable the camera display.
 * @vue-data {boolean} showMap - Enable or disable the map display.
 * @vue-data {boolean} enableJoystick - Enable or disable the joystick ability to send data.
 */

/* Disabled comment documentation
 * Might use those eventually by forking jsdoc-vue-js so it can manage the author
 * and version tag correctly
 * @author Edouard Legare <edouard.legare@usherbrooke.ca>
 * @version 1.0.0
 */

import Vue from 'vue';

import VideoBox from '../widget/VideoBox';
import Joystick from '../widget/Joystick';

export default {
  name: 'teleop-page',
  components: {
    VideoBox,
    Joystick,
  },
  props: {
    bus: {
      type: Vue,
      required: true,
    },
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
  /**
   * Lifecycle Hook - mounted
   *
   * @method
   * @listens mount(el)
   */
  mounted() {
    console.log('Teleop have been mounted');
    this.bus.$emit('mounted');
    this.bus.$on('on-joystick-state-changed', this.changeJoystickState);
  },
  /**
   * Lifecycle Hook - destroyed
   *
   * @method
   * @listens destroyed(el)
   */
  destroyed() {
    console.log('Teleop have been destroyed');
    this.router.$emit('destroyed');
  },
  methods: {
    /**
     * Callback used to change the state of the joystick
     *
     * @method
     * @param {boolean} state - Request of the joystick state
     * @listens on-joystick-state-changed
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
