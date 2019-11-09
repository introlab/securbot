<template>
  <!-- Teleop Page -->
  <b-jumbotron
    id="teleop-layout"
    :fluid="true"
    :container-fluid="true"
    class="h-100 bg-transparent"
  >
    <b-container
      fluid
      class="h-100 w-100 m-auto position-relative"
    >
      <div
        v-if="isConnected"
        class="position-absolute overlay-container"
      >
        <div
          id="general-button-container"
          class="overlay-button-container"
        >
          <!-- Switch Video -->
          <b-button
            id="switch-video-button"
            squared
            class="overlay-button"
            @click="switchHTMLIds"
          >
            Swt
          </b-button>
          <b-tooltip
            target="switch-video-button"
            placement="left"
            variant="secondary"
          >
            {{ (mainVideoId === mapId ? 'Switch to Camera Stream' : 'Switch to Map Stream') }}
          </b-tooltip>
          <!-- Hide Video -->
          <b-button
            id="hide-stream-button"
            squared
            class="overlay-button"
            @click="switchStreamState"
          >
            On
          </b-button>
          <b-tooltip
            target="hide-stream-button"
            placement="left"
            variant="secondary"
          >
            {{ (showStream ? 'Hide Streams' : 'Show Streams') }}
          </b-tooltip>
          <!-- Joystick -->
          <b-button
            id="joystick-enable-button"
            squared
            class="overlay-button"
            :pressed.sync="joystickOverlayEnabled"
            :disabled="!isDataChannelAvailable || gotoOverlayEnabled"
            @click="changeJoystickState"
          >
            J
          </b-button>
          <b-tooltip
            target="joystick-enable-button"
            placement="left"
            variant="secondary"
          >
            {{ (joystickOverlayEnabled ? 'Deactivate Joystick' : 'Activate Joystick') }}
          </b-tooltip>
        </div>
        <div
          id="map-button-container"
          class="overlay-button-container"
        >
          <!-- Zoom Map -->
          <b-button
            id="increase-zoom-button"
            squared
            class="overlay-button"
            :disabled="mainVideoId !== mapId"
            @click="increaseZoom"
          >
            +
          </b-button>
          <b-tooltip
            target="increase-zoom-button"
            placement="left"
            variant="secondary"
          >
            Increase Map Zoom
          </b-tooltip>
          <!-- Unzoom Map -->
          <b-button
            id="decrease-zoom-button"
            squared
            class="overlay-button"
            :disabled="mainVideoId !== mapId"
            @click="decreaseZoom"
          >
            -
          </b-button>
          <b-tooltip
            target="decrease-zoom-button"
            placement="left"
            variant="secondary"
          >
            Decrease Map Zoom
          </b-tooltip>
          <!-- GoTo -->
          <b-button
            id="goto-button"
            squared
            class="overlay-button"
            :disabled="mainVideoId !== mapId || joystickOverlayEnabled"
            :pressed.sync="gotoOverlayEnabled"
            @click="updateMainElement"
          >
            Go
          </b-button>
          <b-tooltip
            target="goto-button"
            placement="left"
            variant="secondary"
          >
            {{ (gotoOverlayEnabled ? 'Cancel GoTo...' : 'Send a GoTo Request') }}
          </b-tooltip>
        </div>
        <div
          id="camera-button-container"
          class="overlay-button-container"
        >

        </div>
      </div>
      <div class="h-100 w-100 m-auto position-relative">
        <!-- Camera -->
        <video-box
          :show="showStream"
          :video-id="mainVideoId"
        />
        <waypoint-overlay
          :is-active="gotoOverlayEnabled"
          :is-clickable="true"
          :show="true"
          :list="demoWP"
          :nb-of-waypoint="1"
          :video-element="mainVideoElement"
          @newWaypoint="sendGoTo"
        />
      </div>
      <div
        class="position-absolute overlay-video-container shadow-sb"
      >
        <video-box
          :show="showStream"
          :video-id="overlayVideoId"
        />
      </div>
      <div
        v-if="joystickOverlayEnabled"
        class="position-absolute overlay-joystick-container shadow-sb"
      >
        <joystick
          :enable="joystickEnabled"
          :absolute-max-x="1"
          :absolute-max-y="1"
          @onPosition="sendJoystickPosition"
        />
      </div>
    </b-container>
  </b-jumbotron>

  <!--  -->
</template>


<script>
import { mapState } from 'vuex';
import VideoBox from '../widgets/VideoBox';
import Joystick from '../widgets/Joystick';
import WaypointOverlay from '../generic/WaypointOverlay';

/**
 * The teleoperation page. Allow an operator to control a robot through the joystick and see what
 * the robot sees.
 *
 * Authors:
 *
 *    - Edouard Legare - <edouard.legare@usherbrooke.ca>,
 * @since 0.1.0
 * @version 1.0.0
 * @displayName Teleoperation View
 */
export default {
  name: 'teleop',
  components: {
    VideoBox,
    Joystick,
    WaypointOverlay,
  },
  data() {
    return {
      showStream: true,
      mainVideoId: '',
      overlayVideoId: '',
      mainVideoElement: '',
      gotoOverlayEnabled: false,
      joystickOverlayEnabled: false,
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
      demoWP: [],
    };
  },
  computed: {
    ...mapState({
      cameraId: state => state.htmlElement.cameraId,
      mapId: state => state.htmlElement.mapId,
      waypointList: state => state.patrol.waypointList,
      joystickEnabled: state => state.joystickEnabled,
      isConnected: state => state.client.connectionState.robot === 'connected',
      isDataChannelAvailable: state => state.client.isDataChannelAvailable,
    }),
  },
  mounted() {
    console.log('Teleop have been mounted');

    this.mainVideoId = this.mapId;
    this.overlayVideoId = this.cameraId;

    this.mainVideoElement = document.getElementById(this.mainVideoId);
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
    demo(event) {
      this.demoWP[0] = event;
    },
    increaseZoom() {
      this.updateMainElement();
      this.$store.dispatch('sendChangeMapZoom', 'increase');
    },
    decreaseZoom() {
      this.updateMainElement();
      this.$store.dispatch('sendChangeMapZoom', 'decrease');
    },
    switchStreamState() {
      this.showStream = !this.showStream;
      this.mainVideoElement = document.getElementById(this.mainVideoId);
      this.$store.dispatch('updateHTMLVideoElements');
    },
    updateMainElement() {
      this.mainVideoElement = document.getElementById(this.mainVideoId);
    },
    sendGoTo(event) {
      console.log('Send Go To');
      this.gotoOverlayEnabled = false;
      this.$store.dispatch('sendGoTo', event);
    },
    switchHTMLIds() {
      if (this.mainVideoId === this.mapId) {
        this.mainVideoId = this.cameraId;
        this.overlayVideoId = this.mapId;
      } else {
        this.mainVideoId = this.mapId;
        this.overlayVideoId = this.cameraId;
      }
      this.gotoOverlayEnabled = false;
      this.$nextTick(() => {
        this.mainVideoElement = document.getElementById(this.mainVideoId);
        this.$store.dispatch('updateHTMLVideoElements');
      });
    },
    /**
     * Enables/disables the sending of joystick data.
     *
     * @public
     * @param {Event} event The event emitted by the toggle.
     */
    changeJoystickState() {
      if (this.joystickOverlayEnabled) {
        this.$store.commit('enableJoystick');
      } else {
        this.$store.commit('disableJoystick');
        this.$store.dispatch('stopTeleop');
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
    sendJoystickPosition(event) {
      this.$store.dispatch('sendJoystickPosition', event);
    },
  },
};
</script>

<style>
.overlay-button {
  background-color: #b5b5b5;
  opacity: 40%;
  height: 60px !important;
  width: 60px !important;
}
.overlay-button:active {
  opacity: 100%;
  border: 2px;
  border-color: black;
}
.overlay-button:disabled {
  opacity: 20%;
  background-color: grey;
}
.overlay-button-container {
  padding: 7px;
  background-color: rgba(194, 194, 194, 0.5);
  border: solid;
  border-color: black;
  border-radius: 5px;
  margin: auto;
  margin-bottom: 5px;
}
.overlay-button-group {
  border: 2px;
  border-color: gray;
}
.overlay-video-container {
  z-index: 100;
  opacity: 60%;
  bottom: 20px;
  left: 40px;
  width: 320px;
  max-width: 320px;
  height: 240px;
  max-height: 240px;
}
.overlay-joystick-container {
  z-index: 100;
  opacity: 50%;
  bottom: 0px;
  right: 70px;
  width: 300px;
  max-width: 300px;
  height: 300px;
  max-height: 300px;
}
.overlay-container {
  top: 20px;
  right: 15px;
  z-index: 100;
  max-width: 80px;
  height: calc( 100% - 40px );
}
</style>
