<template>
  <!-- Teleop Page -->
  <b-jumbotron
    id="teleop-layout"
    :fluid="true"
    :container-fluid="true"
    class="h-100 bg-transparent"
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
            :show="showStream"
            :video-id="cameraId"
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
              :show="showStream"
              :video-id="mapId"
            />
          </div>
        </b-row>
        <!-- Joystick Switch -->
        <div
          class="position-absolute"
          style="top:55%;right:25px;z-index:10;"
        >
          <toggle-button
            :value="joystickEnabled"
            :color="switchColor"
            :sync="true"
            :labels="true"
            :disabled="!connectedToRobot"
            @change="changeJoystickState"
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
                :enable="joystickEnabled"
                :absolute-max-x="1"
                :absolute-max-y="1"
              />
            </div>
          </div>
        </b-row>
      </b-col>
    </b-row>
  </b-jumbotron>
</template>


<script>
import { ToggleButton } from 'vue-js-toggle-button';
import { mapState } from 'vuex';
import VideoBox from '../widgets/VideoBox';
import Joystick from '../widgets/Joystick';

/**
 * The teleoperation page. Allow an operator to control a robot through the joystick and see what
 * the robot sees.
 *
 * Authors:
 *
 *    - Edouard Legare - <edouard.legare@usherbrooke.ca>,
 * @version 2.0.0
 * @displayName Teleoperation View
 */
export default {
  name: 'teleop',
  components: {
    VideoBox,
    Joystick,
    ToggleButton,
  },
  data() {
    return {
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
  computed: mapState({
    cameraId: state => state.htmlElement.cameraId,
    mapId: state => state.htmlElement.mapId,
    showStream: state => state.showStreams,
    joystickEnabled: state => state.joystickEnabled,
    connectedToRobot: state => state.client.connectionState.robot === 'connected',
  }),
  mounted() {
    console.log('Teleop have been mounted');

    this.$store.dispatch('updateHTMLVideoElements');

    this.$nextTick(() => {
      window.addEventListener('resize', this.setJoystickStyle);
    });

    this.setJoystickStyle();
  },
  destroyed() {
    console.log('Teleop have been destroyed');
    window.removeEventListener('resize', this.setJoystickStyle);
  },
  methods: {
    /**
     * Enables/disables the sending of joystick data.
     *
     * @public
     * @param {Event} event The event emitted by the toggle.
     */
    changeJoystickState(event) {
      if (event.value) {
        this.$store.commit('enableJoystick');
      } else {
        this.$store.commit('disableJoystick');
      }
    },
    /**
     * Dynamically sets the joystick height and width to fit the page size.
     * @public
     */
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
