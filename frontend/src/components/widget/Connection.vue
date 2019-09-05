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
          v-for="peer in peersTable"
          :key="peer.peerId"
          type="button"
          class="list-group-item-action list-group-item d-flex justify-content-between
          align-items-center peer-item"
          @click="handlePeerConnection(peer.peerId)"
        >
          {{ peer.peerName }}
          <!-- Tag -->
          <span
            v-if="peer.peerId == isConnectedToPeerId && waitingForConnectionState"
            class="spinner-border spinner-border-sm text-warning"
          />
          <span
            v-else-if="peer.peerId == isConnectedToPeerId"
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
 * @vue-data {Number} isConnectedToPeerId - Peer Id, name should be change...
 * @vue-data {Boolean} waitingForConnectionState - Indicate if connection in progress.
 */

/**
 * Author(s):  Edouard Legare <edouard.legare@usherbrooke.ca>,
 *             Anthony Parris <anthony.parris@usherbrooke.ca>,
 * @version 1.0.0
 */

import Vue from 'vue';

export default {
  name: 'connection',
  props: {
    selfId: {
      type: Number,
      required: true,
    },
    peersTable: {
      type: Array,
      required: true,
    },
    bus: {
      type: Vue,
      required: true,
    },
  },
  data() {
    return {
      isConnected: false,
      isConnectedToPeerId: null,
      waitingForConnectionState: false,
    };
  },
  // On component mounted, get html elements, set bus event.
  mounted() {
    this.bus.$on('connection-changed', this.handleConnectionChanged);
  },
  // On component destroyed, not use for now.
  destroyed() {},
  methods: {
    /**
     * Callback the connection-changed event.
     * @method
     * @param {String} state - New state of the connection.
     * */
    handleConnectionChanged(state) {
      console.log(`Got event for connection : ${state}`);
      switch (state) {
        case 'connected':
          console.log('Connected!');
          this.waitingForConnectionState = false;
          this.isConnected = true;
          // this.switchPeerConnectionButtonHtmlState(this.isConnectedToPeerId, "Connected");
          break;
        case 'failed':
          // Popup : Connection Failed...
          console.log('Connection failed...');
          this.waitingForConnectionState = false;
          this.isConnectedToPeerId = null;
          break;
        case 'disconnected':
          console.log('Disconnected!');
          this.isConnected = false;
          // this.switchPeerConnectionButtonHtmlState(this.isConnectedToPeerId, "Disconnected");
          this.isConnectedToPeerId = null;
          break;
        case 'lost':
          // This Id might not be available anymore... Might get a error...
          // Popup : The connection was lost...
          console.log('Connection lost...');
          this.isConnectedToPeerId = null;
          this.waitingForConnectionState = false;
          this.isConnectedToPeerId = null;
          break;
        default:
          console.log(`Something Something : ${state}`);
      }
    },
    /**
     * Ask to be connected to a peer by emitting event.
     * @method
     * @param {Number} peerId - Peer Id to connect to.
     */
    connectToPeer(peerId) {
      console.log(`Connecting to : ${peerId}`);
      this.isConnectedToPeerId = peerId;
      this.waitingForConnectionState = true;
      this.bus.$emit('peer-connection', peerId);
      console.log('Connecting...');
    },
    /**
     * Ask to be disconnect from the current connected peer by emitting an event.
     * @method
     * @param {Number} peerId - Peer Id currently connected to.
     */
    disconnectFromPeer(peerId) {
      this.bus.$emit('peer-connection', peerId);
    },
    /**
     * HTML button callback to handle the connection/disconnection to a peer.
     * @method
     * @param {Number} peerId - Peer Id associated with the button.
     */
    handlePeerConnection(peerId) {
      if (this.isConnected && peerId === this.isConnectedToPeerId) {
        console.log('Disconnecting...');
        this.disconnectFromPeer(this.isConnectedToPeerId);
      } else if (this.isConnected) {
        // Add popup message saying : "You are already connected to this.isConnectedToPeerId,
        // disconnect from it to connect to an other robot"
        console.log('Already connected to someone...');
      } else if (this.waitingForConnectionState) {
        // Popup saying : "Currently trying to connect to this.isConnectedToPeerId,
        // please be patient..."
        console.log('Waiting for state...');
      } else {
        this.connectToPeer(peerId);
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
