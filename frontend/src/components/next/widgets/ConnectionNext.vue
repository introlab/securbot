<template>
  <!-- Connection Widget -->
  <div
    id="connection"
    class="mx-auto"
    style="min-width:320px"
  >
    <!-- Connection Container -->
    <div
      id="connection-container"
      class="rounded-lg"
      style="border: 1px solid lightgray;"
    >
      <!-- Who Am I ? -->
      <h4
        id="who-am-i"
        class="text-muted ml-1"
      >
        I am : {{ selfId }}
      </h4>
      <!-- Title -->
      <h5 class="ml-1">
        List of Robots:
      </h5>
      <!-- Container list -->
      <div class="list-group">
        <!-- Create a button per robot in room -->
        <button
          v-for="robot in robotList"
          :key="robot.robotId"
          type="button"
          class="list-group-item-action list-group-item d-flex justify-content-between
          align-items-center peer-item"
          @click="handleConnection(robot.robotId)"
        >
          {{ robot.robotName }}
          <!-- Tag -->
          <span
            v-if="robot.robotId === robotId && connectionState === 'connecting'"
            class="spinner-border spinner-border-sm text-warning"
          />
          <span
            v-else-if="robot.robotId == robotId"
            class="badge badge-success"
          >
            Connected
          </span>
          <span
            v-else
            class="badge badge-secondary"
          >
            Not Connected
          </span>
        </button>
      </div>
    </div>
  </div>
</template>

<script>
/**
 * Vue SFC used as a widget that shows the element in the array
 * given in props in a html list. All list element are buttons
 * that, when clicked, request a connection/emit an event on the
 * bus (given in props). It manages the different state on the
 * connection and change a badge to give visual feedback to the
 * user on it. The array given should contains the name and the id
 * of the easyRTC peers that are in the connected room. The component
 * calling this one should manage the easyRTC part and the connection,
 * this component only show peers and manage the connection selection.
 * This component have the following dependencies : Bootstrap-Vue for styling.
 *
 * @module widget/Connection
 * @vue-prop {Number} seflId - Application's easyrtc Id.
 * @vue-prop {String[]} peersTable - Array of robot in the room.
 * @vue-prop {Vue} bus - Vue bus use to emit event to other components.
 * @vue-event {Number} peer-connection - Send the peer id to Layout for connection state change.
 * @vue-data {Boolean} isConnected - Current connection state, only one connection is allowed.
 * @vue-data {Boolean} waitingForConnectionState - Indicate if connection in progress.
 */

/**
 * Author(s):  Edouard Legare <edouard.legare@usherbrooke.ca>,
 *             Anthony Parris <anthony.parris@usherbrooke.ca>,
 * @version 1.0.0
 */
import { mapState } from 'vuex';

/**
 * Remove bus -> Use easyrtc to connect directly or use the store.
 */

export default {
  name: 'connection',
  props: {
    selfId: {
      type: String,
      required: true,
    },
    robotList: {
      type: Array,
      required: true,
    },
  },
  computed: mapState({
    isConnected: state => state.isConnected,
    robotId: state => state.robotId,
    connectionState: state => state.connectionState.robot,
  }),
  methods: {
    /**
     * Callback the connection-changed event.
     * @method
     * @param {String} state - New state of the connection.
     * */
    // handleConnectionChanged(state) {
    //   console.log(`Got event for connection : ${state}`);
    //   switch (state) {
    //     case 'connected':
    //       console.log('Connected!');
    //       this.$store.commit('connectedToRobot');
    //       this.$store.commit('connected');
    //       break;
    //     case 'failed':
    //       // Popup : Connection Failed...
    //       console.log('Connection failed...');
    //       this.$store.commit('failedToConnectToRobot');
    //       this.$store.commit('resetRobotId');
    //       break;
    //     case 'disconnected':
    //       console.log('Disconnected!');
    //       this.$store.commit('disconnectedFromRobot');
    //       this.$store.commit('resetRobotId');
    //       break;
    //     case 'lost':
    //       // This Id might not be available anymore... Might get a error...
    //       // Popup : The connection was lost...
    //       console.log('Connection lost...');
    //       this.$store.commit('failedToConnectToRobot');
    //       this.$store.commit('resetRobotId');
    //       break;
    //     default:
    //       console.log(`This should not happen/isn't managed : ${state}`);
    //   }
    // },
    /**
     * Ask to be connected to a peer by emitting event.
     * @method
     * @param {Number} peerId - Peer Id to connect to.
     */
    // connectToRobot(robotId) {
    //   console.log(`Connecting to : ${robotId}`);
    //   this.$store.commit('setRobotId', robotId);
    //   this.$store.commit('connectingToRobot');
    //   this.$store.dispatch('connectToRobot');
    //   console.log('Connecting...');
    // },
    /**
     * Ask to be disconnect from the current connected peer by emitting an event.
     * @method
     * @param {Number} peerId - Peer Id currently connected to.
     */
    // disconnectFromRobot(robotId) {
    //   this.bus.$emit('peer-connection', robotId);
    // },
    /**
     * HTML button callback to handle the connection/disconnection to a peer.
     * @method
     * @param {Number} peerId - Peer Id associated with the button.
     */
    handleConnection(robotId) {
      if (this.isConnected && robotId === this.robotId) {
        console.log('Disconnecting...');
        this.$store.dispatch('disconnectFromRobot'); // disconnectFromRobot(this.robotId);
      } else if (this.isConnected && this.robotId) {
        // Add popup message saying : "You are already connected to this.robotId,
        // disconnect from it to connect to an other robot"
        console.log('Already connected to someone...');
      } else if (this.connectionState === 'connecting') {
        // Popup saying : "Currently trying to connect to this.robotId,
        // please be patient..."
        console.log('Waiting for state...');
      } else {
        this.$store.dispatch('connectToRobot', robotId); // .connectToRobot(robotId);
      }
    },
  },
};
</script>

<style>
/* Size for list item */
.peer-item{
  min-width: 300px;
  min-height: 30px;
}
</style>
