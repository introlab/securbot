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
      <div class="position-relative main-video-container shadow-sb">
        <video-box
          :show="showStream"
          :zoom="(mainVideoId === mapId ? mapZoom : 1)"
          :video-id="mainVideoId"
        />
        <waypoint-overlay
          :is-active="gotoOverlayEnabled"
          :is-clickable="true"
          :show="true"
          :show-grid="showGrid"
          :list="demoWP"
          :zoom="mapZoom"
          :map-size="mapSize"
          :nb-of-waypoint="1"
          wp-color="#00d456"
          :video-element="mainVideoElement"
          @newWaypoint="sendGoTo"
        />
      </div>
      <div
        class="position-absolute overlay-video-container shadow-sb"
      >
        <video-box
          :show="showStream"
          :zoom="(mainVideoId === mapId ? 1 : mapZoom)"
          :video-id="overlayVideoId"
        />
      </div>
      <!-- BUTTONS -->
      <div
        v-if="isConnected"
        class="overlay-container-teleop"
      >
        <div
          id="teleop-overlay-button-container"
          class="overlay-button-container"
        >
          <!-- Switch Video -->
          <b-button
            id="switch-video-button"
            squared
            class="overlay-button"
            @click="switchHTMLIds"
          >
            <font-awesome-icon
              class="icon-button-teleop"
              icon="exchange-alt"
            />
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
            <font-awesome-icon
              v-if="showStream"
              class="icon-button-teleop"
              icon="eye-slash"
            />
            <font-awesome-icon
              v-else
              class="icon-button-teleop"
              icon="eye"
            />
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
            :class="{ 'overlay-button-active': joystickOverlayEnabled }"
            :pressed.sync="joystickOverlayEnabled"
            :disabled="!isDataChannelAvailable || gotoOverlayEnabled || keyboardCtrl.enabled"
            @click="changeJoystickState"
          >
            <font-awesome-icon
              class="icon-button-teleop"
              icon="gamepad"
            />
          </b-button>
          <b-tooltip
            target="joystick-enable-button"
            placement="left"
            variant="secondary"
          >
            {{ (joystickOverlayEnabled ? 'Deactivate Joystick' : 'Activate Joystick') }}
          </b-tooltip>
          <!-- Keyboard -->
          <b-button
            id="keyboard-enable-button"
            squared
            class="overlay-button"
            :class="{ 'overlay-button-active': keyboardCtrl.enabled }"
            :pressed.sync="keyboardCtrl.enabled"
            :disabled="!isDataChannelAvailable || gotoOverlayEnabled || joystickOverlayEnabled"
            @click="sendKeyboardLastCommand"
          >
            <font-awesome-icon
              class="icon-button-teleop"
              icon="keyboard"
            />
          </b-button>
          <b-tooltip
            target="keyboard-enable-button"
            placement="left"
            variant="secondary"
          >
            {{ (joystickOverlayEnabled
              ? 'Deactivate Keyboard Control'
              : 'Activate Keyboard Control') }}
          </b-tooltip>
          <!-- Dock -->
          <b-button
            id="dock-request-button"
            squared
            class="overlay-button"
            :class="{ 'overlay-button-active': dockingEnabled }"
            :active="dockingEnabled"
            :disabled="!isDataChannelAvailable"
            @click="configDockingProcess"
          >
            <font-awesome-icon
              class="icon-button-teleop"
              icon="plug"
            />
          </b-button>
          <b-tooltip
            target="dock-request-button"
            placement="left"
            variant="secondary"
          >
            {{ (dockingEnabled
              ? 'Stop Docking Process'
              : 'Start Docking Process') }}
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
            <font-awesome-icon
              class="icon-button-teleop"
              icon="plus"
            />
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
            <font-awesome-icon
              class="icon-button-teleop"
              icon="minus"
            />
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
            :class="{ 'overlay-button-active': gotoOverlayEnabled }"
            :disabled="mainVideoId !== mapId || joystickOverlayEnabled || keyboardCtrl.enabled"
            :pressed.sync="gotoOverlayEnabled"
            @click="updateMainElement"
          >
            <font-awesome-icon
              class="icon-button-teleop"
              icon="map-marker-alt"
            />
          </b-button>
          <b-tooltip
            target="goto-button"
            placement="left"
            variant="secondary"
          >
            {{ (gotoOverlayEnabled ? 'Cancel GoTo...' : 'Send a GoTo Request') }}
          </b-tooltip>
        </div>
        <!-- <div
          id="camera-button-container"
          class="overlay-button-container"
        /> -->
        <div
          v-if="joystickOverlayEnabled || keyboardCtrl.enabled"
          id="teleop-control-container"
          class="overlay-button-container"
        >
          <b-button
            id="increase-teleop-gain-button"
            squared
            class="overlay-button"
            @click="increaseGain"
          >
            <font-awesome-icon
              class="icon-button-teleop"
              icon="plus"
            />
          </b-button>
          <b-tooltip
            target="increase-teleop-gain-button"
            placement="left"
            variant="secondary"
          >
            Increase Teleop Gain ({{ (teleopGain * 100).toFixed(0) }}%)
          </b-tooltip>
          <b-button
            id="decrease-teleop-gain-button"
            squared
            class="overlay-button"
            @click="decreaseGain"
          >
            <font-awesome-icon
              class="icon-button-teleop"
              icon="minus"
            />
          </b-button>
          <b-tooltip
            target="decrease-teleop-gain-button"
            placement="left"
            variant="secondary"
          >
            Decrease Teleop Gain ({{ (teleopGain * 100).toFixed(0) }}%)
          </b-tooltip>
        </div>
      </div>
      <div
        v-if="joystickOverlayEnabled"
        class="overlay-joystick-container"
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
      teleopGain: 0.5,
      keyboardCtrlSendTime: 100,
      keyboardCtrlInterval: '',
      keyboardCtrlKeys: {
        up: false,
        down: false,
        left: false,
        right: false,
      },
      keyboardCtrl: {
        x: 0,
        y: 0,
        enabled: false,
      },
      showStream: true,
      showGrid: process.env.NODE_ENV !== 'production',
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
      mapZoom: state => state.mapZoom,
      mapSize: state => state.mapSize,
      cameraId: state => state.htmlElement.cameraId,
      mapId: state => state.htmlElement.mapId,
      joystickEnabled: state => state.joystickEnabled,
      isConnected: state => state.client.connectionState.robot === 'connected',
      isDataChannelAvailable: state => state.client.isDataChannelAvailable,
      dockingEnabled: state => !!state.dockingInterval,
    }),
  },
  mounted() {
    this.mainVideoId = this.mapId;
    this.overlayVideoId = this.cameraId;

    this.$nextTick(() => {
      this.updateMainElement();
      this.$store.dispatch('updateHTMLVideoElements');
    });

    document.addEventListener('keydown', this.onKeydown);
    document.addEventListener('keyup', this.onKeyup);
    this.keyboardCtrlInterval = setInterval(this.sendKeyboardControl, this.keyboardCtrlSendTime);
  },
  destroyed() {
    this.$store.commit('disableJoystick');
    this.$store.dispatch('stopTeleop');
    clearInterval(this.keyboardCtrlInterval);
    document.removeEventListener('keydown', this.onKeydown);
    document.removeEventListener('keyup', this.onKeyup);
  },
  methods: {
    demo(event) {
      this.demoWP[0] = event;
    },
    increaseGain() {
      this.teleopGain += 0.05;
      this.saturateGain();
    },
    decreaseGain() {
      this.teleopGain -= 0.05;
      this.saturateGain();
    },
    saturateGain() {
      this.teleopGain = Math.round(this.teleopGain * 100) / 100;
      if (this.teleopGain > 1) {
        this.teleopGain = 1;
      } else if (this.teleopGain < 0) {
        this.teleopGain = 0;
      }
    },
    sendKeyboardControl() {
      if (this.keyboardCtrl.enabled) {
        // eslint-disable-next-line max-len
        this.keyboardCtrl.x = (this.keyboardCtrlKeys.left ? -this.teleopGain : 0) + (this.keyboardCtrlKeys.right ? this.teleopGain : 0);
        // eslint-disable-next-line max-len
        this.keyboardCtrl.x = Number(this.keyboardCtrl.x.toFixed(2));
        // eslint-disable-next-line max-len
        this.keyboardCtrl.y = (this.keyboardCtrlKeys.up ? -this.teleopGain : 0) + (this.keyboardCtrlKeys.down ? this.teleopGain : 0);
        // eslint-disable-next-line max-len
        this.keyboardCtrl.y = Number((Math.abs(this.keyboardCtrl.y) * this.keyboardCtrl.y).toFixed(2));
        this.$store.dispatch('sendJoystickPosition', this.keyboardCtrl);
      }
    },
    onKeydown(event) {
      if (!event.repeat && this.keyboardCtrl.enabled) {
        switch (event.code) {
          case 'KeyW':
          case 'ArrowUp':
            this.keyboardCtrlKeys.up = true;
            break;
          case 'KeyS':
          case 'ArrowDown':
            this.keyboardCtrlKeys.down = true;
            break;
          case 'KeyA':
          case 'ArrowLeft':
            this.keyboardCtrlKeys.left = true;
            break;
          case 'KeyD':
          case 'ArrowRight':
            this.keyboardCtrlKeys.right = true;
            break;
          default:
            break;
        }
      }
    },
    onKeyup(event) {
      if (!event.repeat && this.keyboardCtrl.enabled) {
        switch (event.code) {
          case 'KeyW':
          case 'ArrowUp':
            this.keyboardCtrlKeys.up = false;
            break;
          case 'KeyS':
          case 'ArrowDown':
            this.keyboardCtrlKeys.down = false;
            break;
          case 'KeyA':
          case 'ArrowLeft':
            this.keyboardCtrlKeys.left = false;
            break;
          case 'KeyD':
          case 'ArrowRight':
            this.keyboardCtrlKeys.right = false;
            break;
          default:
            break;
        }
      }
    },
    configDockingProcess() {
      console.log('Docking Process...');
      if (this.dockingEnabled) {
        this.$store.dispatch('stopDockingProcess');
      } else {
        this.$store.dispatch('startDockingProcess');
      }
    },
    increaseZoom() {
      this.updateMainElement();
      this.$store.commit('increaseMapZoom');
    },
    decreaseZoom() {
      this.updateMainElement();
      this.$store.commit('decreaseMapZoom');
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
    sendKeyboardLastCommand() {
      if (!this.keyboardCtrl.enabled) {
        this.$store.dispatch('stopTeleop');
      }
    },
    sendJoystickPosition(event) {
      const pos = {};
      Object.assign(pos, event);
      pos.x = Number(((Math.abs(pos.x) * pos.x) * this.teleopGain).toFixed(2));
      pos.y = Number(((Math.abs(pos.y) * pos.y) * this.teleopGain).toFixed(2));
      this.$store.dispatch('sendJoystickPosition', pos);
    },
  },
};
</script>

<style scoped>
.icon-button-teleop {
  color: black;
  height: 10px;
  width: 10px;
}
.overlay-button {
  background-color: grey;
  opacity: 0.4;
  padding: 0rem;
  height: 25px !important;
  width: 25px !important;
}
.main-video-container {
  height: 40%;
  width: 100%;
  margin: auto;
}
.overlay-video-container {
  z-index: 100;
  opacity: 0.6;
  width: 0px;
  max-width: 0px;
  height: 0px;
  max-height: 0px;
}
.overlay-joystick-container {
  position: relative;
  opacity: 0.5;
  width: 100%;
  max-width: 100%;
  height: 50%;
  max-height: 50%;
}
.overlay-container-teleop {
  margin-top: 0rem;
  position: relative;
  display: flex;
}
.overlay-button-container {
  display: flex;
  padding: 7px;
  background-color: rgba(25, 25, 25, 0.75);
  border-radius: 0 0 5px 5px;
  margin: auto;
  margin-bottom: 5px;
}
#keyboard-enable-button {
  display: none;
}
@media (min-width: 800px) and (min-height: 600px) {
  #keyboard-enable-button {
    display: block;
  }
  .icon-button-teleop {
    color: white;
    height: 10px;
    width: 10px;
  }
  .overlay-button {
    background-color: #b5b5b5;
  }
  .overlay-container-teleop {
    display: block;
    margin-top: 0px;
    position: absolute;
    max-width: 45px;
    top: 5px;
    right: 15px;
    z-index: 100;
    height: calc( 100% - 40px );
  }
  .overlay-button-container {
    display: block;
    padding: 7px;
    background-color: rgba(245, 245, 245, 0.75);
    border-radius: 5px 0 0 5px;
    margin: auto;
    margin-bottom: 5px;
  }
  .main-video-container {
    height: 100%;
  }
  .overlay-video-container {
    bottom: 5px;
    left: 20px;
    width: 200px;
    max-width: 200px;
    height: 150px;
    max-height: 150px;
  }
  .overlay-joystick-container {
    position: absolute;
    z-index: 100;
    opacity: 0.5;
    bottom: 0px;
    right: 50px;
    width: 200px;
    max-width: 200px;
    height: 200px;
    max-height: 200px;
  }
}
@media (min-width: 1366px) and (min-height: 768px) {
  .icon-button-teleop {
    color: white;
    height: 15px;
    width: 15px;
  }
  .overlay-button {
    padding: 0.5rem;
    height: 45px !important;
    width: 45px !important;
  }
  .overlay-container-teleop {
    max-width: 60px;
  }
  .overlay-joystick-container {
    right: 70px;
    width: 300px;
    max-width: 300px;
    height: 300px;
    max-height: 300px;
  }
}
@media (min-width: 1600px) and (min-height: 900px) {
  .icon-button-teleop {
    height: 20px;
    width: 20px;
  }
  .overlay-button {
    opacity: 0.4;
    height: 60px !important;
    width: 60px !important;
  }
  .overlay-container-teleop {
    max-width: 80px;
  }
  .overlay-video-container {
    bottom: 20px;
    left: 40px;
    width: 400px;
    max-width: 400px;
    height: 300px;
    max-height: 300px;
  }
  .overlay-joystick-container {
    right: 90px;
    width: 400px;
    max-width: 400px;
    height: 400px;
    max-height: 400px;
  }
}
.overlay-button-active {
  opacity: 0.8;
  border-style: solid;
  border-width: 2px;
  border-color: #00A759 !important;
}
.overlay-button:disabled {
  opacity: 0.2;
  background-color: grey;
}
.overlay-button-group {
  border: 2px;
  border-color: gray;
}
</style>
