<template>
  <!-- Teleop Page -->
  <b-jumbotron
    id="teleop-layout"
    :fluid="true"
    :container-fluid="true"
    class="h-100"
  >
    <!-- Row -->
    <b-row class="h-100">
      <!-- Camera column -->
      <b-col
        lg="8"
        md="7"
        sm="6"
        class="mh-100"
      >
        <!-- Camera Container -->
        <div class="h-100 w-100 m-auto position-relative">
          <!-- Camera -->
          <video-box
            :show="showCamera"
            video-id="camera-stream"
          />
        </div>
      </b-col>
      <!-- Map and joystick column -->
      <b-col
        lg="4"
        md="5"
        sm="6"
        class="mh-100"
      >
        <!-- Map row -->
        <b-row class="h-50 w-100 position-relative m-0">
          <!-- Map container -->
          <div class="h-100 w-100 m-auto position-relative">
            <!-- Map -->
            <video-box
              :show="showMap"
              video-id="map-stream"
            />
          </div>
        </b-row>
        <!-- Joystick Switch -->
        <div
          class="position-absolute"
          style="top:55%;right:25px;z-index:10;"
        >
          <toggle-button
            :value="enableJoystick"
            :color="switchColor"
            :sync="true"
            :labels="true"
            :disabled="disableJoystick"
            @change="enableJoystick = $event.value"
          />
        </div>
        <!-- Joystick Row -->
        <b-row
          id="joystick-row"
          class="position-relative h-50 m-auto p-4"
        >
          <!-- Joystick Outer -->
          <div
            class="position-relative m-auto"
            :style="joystickStyle"
          >
            <!-- Joystick Inner-->
            <div
              class="position-absolute h-100 w-100 border
              border-secondary rounded-circle shadow-sb"
              style="top:0;left:0;"
            >
              <!-- Joystick -->
              <joystick
                :enable="enableJoystick"
                :absolute-max-x="1"
                :absolute-max-y="1"
                :bus="bus"
              />
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

/** Disabled comment documentation
 * Might use those eventually by forking jsdoc-vue-js so it can manage the author
 * and version tag correctly
 * @author Edouard Legare <edouard.legare@usherbrooke.ca>
 * @version 1.0.0
 */

import { ToggleButton } from 'vue-js-toggle-button';
import Vue from 'vue';

import VideoBox from '../widget/VideoBox';
import Joystick from '../widget/Joystick';

export default {
  name: 'teleop-page',
  components: {
    VideoBox,
    Joystick,
    ToggleButton,
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
  /**
   * Lifecycle Hook - mounted
   *
   * @method
   * @listens mount(el)
   */
  mounted() {
    console.log('Teleop have been mounted');
    this.bus.$on('on-joystick-state-changed', this.changeJoystickState);
    this.router.$emit('mounted');

    this.$nextTick(() => {
      window.addEventListener('resize', this.setJoystickStyle);
    });

    this.setJoystickStyle();
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

    window.removeEventListener('resize', this.setJoystickStyle);
  },
  methods: {
    /**
     * Callback used to change the state of the joystick
     * @method
     * @param {boolean} state - Request of the joystick state
     * @listens on-joystick-state-changed
     */
    changeJoystickState(state) { // Change to computed
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
