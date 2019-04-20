<template>
  <b-jumbotron
    id="teleop-layout"
    :fluid="true"
    :container-fluid="true"
    class="h-100 "
    bg-variant="light">
    <b-row class="h-100">
      <!-- Camera Video -->
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
        <!-- Map Video -->
        <b-row class="h-50 w-100 position-relative m-0">
          <div class="h-100 w-100 m-auto position-relative">
            <video-box
              :show="showMap"
              video-id="map-stream" />
          </div>
        </b-row>
        <!-- Joystick -->
        <div
          class="position-absolute"
          style="top:55%;right:25px;z-index:10;">
          <toggle-button
            :value="enableJoystick"
            :color="switchColor"
            :sync="true"
            :labels="true"
            :disabled="disableJoystick"
            @change="enableJoystick = $event.value" />
        </div>
        <b-row
          id="joystick-row"
          class="position-relative h-50 m-auto p-4">
          <div
            class="position-relative m-auto"
            :style="joystickStyle">
            <div
              class="position-absolute h-100 w-100 border
              border-secondary rounded-circle shadow-sb"
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
import { ToggleButton } from 'vue-js-toggle-button';
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
    ToggleButton,
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
      disableJoystick: true,
      joystickStyle: {
        width: '100%',
        'padding-top': '100%',
        height: 0,
      },
      switchColor: {
        checked: '#00A759',
        unchecked: '#808080',
        disabled: '#E8E8E8',
      },
    };
  },
  mounted() {
    console.log('Teleop have been mounted');
    this.bus.$on('on-joystick-state-changed', this.changeJoystickState);
    this.router.$emit('mounted');

    this.$nextTick(() => {
      window.addEventListener('resize', this.setJoystickStyle);
    });

    this.setJoystickStyle();
  },
  destroyed() {
    console.log('Teleop have been destroyed');
    /**
     * Destroyed event
     * @arg Does not take any parameter
     */
    this.router.$emit('destroyed');

    window.removeEventListener('resize', this.setJoystickStyle);
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
        this.disableJoystick = false;
      } else {
        this.enableJoystick = false;
        this.disableJoystick = true;
      }
    },
    setJoystickStyle() {
      let ratio = 1;
      const e = document.getElementById('joystick-row');
      if (e) {
        ratio = ((e.clientHeight / e.clientWidth) < 1)
          ? `${(((e.clientHeight / e.clientWidth) * 100) - 5)}%` : '95%';
        this.joystickStyle = {
          width: ratio,
          'padding-top': ratio,
          height: 0,

        };
      }
    },
  },
};
</script>

<style>
</style>
